//
// Created by msi on 2021/5/18.
//

#ifndef QJAKA_GUI_ROBOT_HPP
#define QJAKA_GUI_ROBOT_HPP

#include <QObject>

#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <functional>
#include <iostream>
#include <random>

#include "tools.hpp"


class VirtualRobot {

protected:
    std::atomic_bool m_isLogin;
    double m_radPerSecond = 5 / 180.0 * M_PI;

private:
    std::mutex m_mutex;
    RobotStatus m_status{};
    std::chrono::high_resolution_clock::time_point m_endTime;
    double m_targetJointVal[6] = {};
    double m_startJointVal[6] = {};
    std::chrono::milliseconds m_duration;
    std::atomic_bool m_isMoving;
    std::atomic_bool m_isCurAbort;


    // !! 线程不安全, 使用前需要上锁
    errno_t m_updateJointVal_unsafe() {
        auto current = std::chrono::high_resolution_clock::now();
        if (current > m_endTime) {
            if (m_isMoving.load()) {
                memcpy(m_status.joint_position, m_targetJointVal, sizeof(m_targetJointVal));
                m_isMoving.exchange(false);
            }
        } else {
            auto t = std::chrono::duration_cast<std::chrono::milliseconds>(m_endTime - current).count();
            auto duration = m_duration.count();
            double radio = (double) t / (double) duration;
            for (int i = 0; i < 6; ++i) {
                m_status.joint_position[i] = m_targetJointVal[i] - (m_targetJointVal[i] - m_startJointVal[i]) * radio;
            }
        }
        if (m_isCurAbort.load()) {
            m_isCurAbort.exchange(false);
            return ERR_CUSTOM_RECV_ABORT;
        }
        return ERR_SUCC;
    }

public:
    static void sleepMilliseconds(long long milliseconds = 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }

    VirtualRobot() : m_isLogin(false), m_isMoving(false), m_isCurAbort(false) {}

//    ~VirtualRobot() {}

    bool is_login() {
        return m_isLogin.load();
    }

    virtual errno_t login_in(const char *ip) {
        sleepMilliseconds();
        m_isLogin.exchange(true);
        std::lock_guard<std::mutex> lock(m_mutex);
        m_status.is_socket_connect = 1;
        return ERR_SUCC;
    }

    virtual errno_t login_out() {
        sleepMilliseconds();
        m_isLogin.exchange(false);
        return ERR_SUCC;
    }

    virtual errno_t power_on() {
        sleepMilliseconds();
        std::lock_guard<std::mutex> lock(m_mutex);
        m_status.powered_on = 1;
        return ERR_SUCC;
    }

    virtual errno_t power_off() {
        sleepMilliseconds();
        std::lock_guard<std::mutex> lock(m_mutex);
        m_status.powered_on = 0;
        return ERR_SUCC;
    }

    virtual errno_t enable_robot() {
        sleepMilliseconds();
        std::lock_guard<std::mutex> lock(m_mutex);
        m_status.enabled = 1;
        return ERR_SUCC;
    }

    virtual errno_t disable_robot() {
        sleepMilliseconds();
        std::lock_guard<std::mutex> lock(m_mutex);
        m_status.enabled = 0;
        return ERR_SUCC;
    }

    void set_spin_speed(double radPerSpeed) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_radPerSecond = radPerSpeed;
    }

    virtual errno_t set_rapidrate(double rapid) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_status.rapidrate = rapid;
        return ERR_SUCC;
    }


    virtual errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode) {
        if (m_isMoving.load()) return ERR_CUSTOM_IS_MOVING;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_status.emergency_stop) return ERR_EMERGENCY_PRESSED;
            else if (!m_status.powered_on) return ERR_NOT_POWERED;
            else if (!m_status.enabled) return ERR_NOT_ENABLED;


            m_isMoving.exchange(true);

            double maxDeltaRad = 0;
            memcpy(m_startJointVal, m_status.joint_position, sizeof(m_startJointVal));
            if (move_mode == INCR) {
                for (int i = 0; i < 6; ++i) {
                    m_targetJointVal[i] = m_startJointVal[i] + joint_pos->jVal[i];
                    maxDeltaRad = std::max(maxDeltaRad, std::fabs(joint_pos->jVal[i]));
                }
            } else if (move_mode == ABS) {
                for (int i = 0; i < 6; ++i) {
                    m_targetJointVal[i] = joint_pos->jVal[i];
                    maxDeltaRad = std::max(maxDeltaRad, std::fabs(m_targetJointVal[i] - m_startJointVal[i]));
                }
            } else {
                return ERR_INVALID_PARAMETER;
            }
            m_duration = std::chrono::milliseconds(int(maxDeltaRad * 1000.0 / m_radPerSecond));
            m_endTime = std::chrono::high_resolution_clock::now() + m_duration;
        }
        while (m_isMoving.load()) {
            sleepMilliseconds(10);
        }
        if (m_isCurAbort.load()) return ERR_CUSTOM_RECV_ABORT;
        else return ERR_SUCC;
    }

    virtual errno_t motion_abort() {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_updateJointVal_unsafe();
        if (m_isMoving.load()) {
            m_endTime = std::chrono::high_resolution_clock::now();
            m_isMoving.exchange(false);
            m_isCurAbort.exchange(true);
        }
        return ERR_SUCC;
    }

    virtual errno_t collision_recover() {
        return ERR_SUCC;
    }

    virtual errno_t get_joint_position(JointValue *jVal) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_updateJointVal_unsafe();
        memcpy(jVal->jVal, m_status.joint_position, sizeof(m_status.joint_position));
        return ERR_SUCC;
    }

    virtual errno_t get_robot_state(RobotState *state) {
        std::lock_guard<std::mutex> lock(m_mutex);
        state->estoped = m_status.emergency_stop;
        state->poweredOn = m_status.powered_on;
        state->servoEnabled = m_status.enabled;
        return ERR_SUCC;
    }

    virtual errno_t get_robot_status(RobotStatus *status) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_updateJointVal_unsafe();
        memcpy(status, &m_status, sizeof(RobotStatus));
        return ERR_SUCC;
    }
};


class RealRobot : public VirtualRobot {
private:
    JAKAZuRobot m_robot;
public:
    RealRobot() {
        char version[1024];
        m_robot.get_sdk_version(version);
        std::cout << "jaka sdk version: " << version << std::endl;
    }

    errno_t login_in(const char *ip) override {
        std::cout << "login real robot" << std::endl;
        auto res = m_robot.login_in(ip);
        std::cout << "login complete" << std::endl;
        if (res == ERR_SUCC) {
            m_isLogin.exchange(true);
//            m_robot.set_network_exception_handle(200, MOT_ABORT);
        }
        return res;
    }

    errno_t login_out() override {
        auto res = m_robot.login_out();
        if (res == ERR_SUCC) m_isLogin.exchange(false);
        return res;
    }

    errno_t power_on() override {
        return m_robot.power_on();
    }

    errno_t power_off() override {
        return m_robot.power_off();

    }

    errno_t enable_robot() override {
        return m_robot.enable_robot();

    }

    errno_t disable_robot() override {
        return m_robot.disable_robot();
    }

    errno_t set_rapidrate(double rapid) override {
        return m_robot.set_rapidrate(rapid);
    }

    errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode) override {
        return m_robot.joint_move(joint_pos, move_mode, true, m_radPerSecond);
    }

    errno_t motion_abort() override {
        return m_robot.motion_abort();
    }

    errno_t collision_recover() override {
        return m_robot.collision_recover();
    }

    errno_t get_joint_position(JointValue *jVal) override {
        return m_robot.get_joint_position(jVal);
    }

    errno_t get_robot_state(RobotState *state) override {
        return m_robot.get_robot_state(state);
    }

    errno_t get_robot_status(RobotStatus *status) override {
        return m_robot.get_robot_status(status);
    }
};


#endif  // QJAKA_GUI_ROBOT_HPP
