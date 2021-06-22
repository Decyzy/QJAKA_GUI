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

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
    std::chrono::milliseconds m_duration{};
    std::atomic_bool m_isMoving;
    std::atomic_bool m_isCurAbort;
    std::atomic_int m_collisionLevel;
    std::string m_prefix;


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

    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;

    void m_updatePose_unsafe() {
        try {
            geometry_msgs::TransformStamped trans = m_tfBuffer.lookupTransform(m_prefix + "base_link",
                                                                               m_prefix + "gripper_base_link",
                                                                               ros::Time(0));
            tf2::Quaternion qua;
            tf2::convert(trans.transform.rotation, qua);
            tf2::Matrix3x3(qua).getRPY(m_status.cartesiantran_position[3],
                                       m_status.cartesiantran_position[4],
                                       m_status.cartesiantran_position[5]);
            m_status.cartesiantran_position[0] = trans.transform.translation.x * 1000.0;
            m_status.cartesiantran_position[1] = trans.transform.translation.y * 1000.0;
            m_status.cartesiantran_position[2] = trans.transform.translation.z * 1000.0;
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR_ONCE_NAMED("qjaka_gui", "%s", ex.what());
            m_status.cartesiantran_position[0] = 0.0;
        }
    }

public:
    static void sleepMilliseconds(long long milliseconds = 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }

    VirtualRobot() : m_isLogin(false), m_isMoving(false), m_isCurAbort(false),
                     m_collisionLevel(1), m_duration(), m_tfListener(m_tfBuffer) {
    }

//    ~VirtualRobot() {}

    bool is_login() {
        return m_isLogin.load();
    }

    virtual bool is_virtual() { return true; }

    void set_prefix(const std::string &prefix) {
        m_prefix = prefix;
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
            sleepMilliseconds(1);
        }
        if (m_isCurAbort.load()) return ERR_CUSTOM_RECV_ABORT;
        else return ERR_SUCC;
    }

    virtual errno_t servo_move_enable(bool enable) {
        return ERR_SUCC;
    }

    virtual errno_t servo_move_use_joint_MMF(int max_buf, double kp, double kv, double ka) {
        return ERR_SUCC;
    }

    virtual errno_t servo_j(const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num) {
        return joint_move(joint_pos, move_mode);
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
        m_updatePose_unsafe();
        memcpy(status, &m_status, sizeof(RobotStatus));
        return ERR_SUCC;
    }

    virtual errno_t get_collision_level(int *level) {
        *level = m_collisionLevel.load();
        return ERR_SUCC;
    }

    virtual errno_t set_collision_level(const int level) {
        m_collisionLevel.exchange(level);
        return ERR_SUCC;
    }

    virtual errno_t set_do(int index, bool on) {
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
//        double pose[6];
//        pose[3] = -1.2;
//        pose[4] = 0.43;
//        pose[5] = 0.771;
//        Eigen::Vector3d eulerAngle(pose[3], pose[4], pose[5]);
//        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
//        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
//        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
//        Eigen::Quaterniond quaternion;
//        quaternion = yawAngle * pitchAngle * rollAngle;
//
//        Rpy rpy;
//        RotMatrix rot_matrix;
//        Quaternion qua;
//        rpy.rx = pose[3];
//        rpy.ry = pose[4];
//        rpy.rz = pose[5];
//        m_robot.rpy_to_rot_matrix(&rpy, &rot_matrix);
//        m_robot.rot_matrix_to_quaternion(&rot_matrix, &qua);
//        qDebug() << qua.x << ", " << qua.y << ", " << qua.z << ", " << qua.s;
//        qDebug() << quaternion.x() << ", " << quaternion.y() << ", " << quaternion.z() << ", " << quaternion.w();
//
//        Eigen::Vector3d angle = quaternion.matrix().eulerAngles(2, 1, 0);
//        qDebug() << angle(2) << ", " << angle(1) << "" << angle(0);

        std::cout << "jaka sdk version: " << version << std::endl;
    }

    bool is_virtual() override { return false; }


    errno_t login_in(const char *ip) override {
        std::cout << "login real robot" << std::endl;
        auto res = m_robot.login_in(ip);
        std::cout << "login complete" << std::endl;
        if (res == ERR_SUCC) {
            auto r = m_robot.set_error_handler([](int errorCode) {
                std::cout << "ERROR!! error code = " << errorCode << std::endl;
            });
            std::cout << "set_error_handler: " << r << std::endl;
            char filepath[] = "/home/msi/Documents/JAKA_SDK_LOG";
            r = m_robot.set_SDK_filepath(filepath);
            std::cout << "set_SDK_filepath: " << res << std::endl;
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

    errno_t servo_move_enable(bool enable) override {
        return m_robot.servo_move_enable(enable ? TRUE : FALSE);
    }

    errno_t servo_move_use_joint_MMF(int max_buf, double kp, double kv, double ka) override {
        return m_robot.servo_move_use_joint_MMF(max_buf, kp, kv, ka);
    }

    errno_t servo_j(const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num) override {
        return m_robot.servo_j(joint_pos, move_mode, step_num);
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

    errno_t get_collision_level(int *level) override {
        return m_robot.get_collision_level(level);
    }

    errno_t set_collision_level(const int level) override {
        return m_robot.set_collision_level(level);
    }

    errno_t set_do(int index, bool on) override {
        return m_robot.set_digital_output(IO_CABINET, index, on ? TRUE : FALSE);
    }
};


#endif  // QJAKA_GUI_ROBOT_HPP
