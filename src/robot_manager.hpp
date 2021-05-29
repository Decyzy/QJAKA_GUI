//
// Created by msi on 2021/5/20.
//

#ifndef JAKA_GUI_ROBOT_MANAGER_HPP
#define JAKA_GUI_ROBOT_MANAGER_HPP

#include <QObject>

#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <functional>
#include <iostream>
#include <random>
#include <utility>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotTrajectory.h>

#include "robot.hpp"


class ROSClient {
private:
    ros::NodeHandle &m_nh;
    sensor_msgs::JointState m_msg;
    ros::Publisher m_jointStatePub;

public:
    const std::string m_prefix;

    explicit ROSClient(ros::NodeHandle &nh, std::string prefix) : m_nh(nh), m_prefix(std::move(prefix)) {
        m_jointStatePub = m_nh.advertise<sensor_msgs::JointState>("jaka_joint_states", 10);

        m_msg = sensor_msgs::JointState();
        m_msg.name.resize(6);
        m_msg.position.resize(6);

        for (int i = 0; i < 6; ++i) {
            m_msg.name[i] = m_prefix + "joint_" + std::to_string(i + 1);
            m_msg.position[i] = 0.0;
        }
    }

    void publish_joint_states(const double states[6]) {
        for (int i = 0; i < 6; ++i) {
            m_msg.position[i] = states[i];
        }
        m_jointStatePub.publish(m_msg);
    }
};


class StackThread {
private:
    std::thread m_t;
    std::function<void(void)> m_func = nullptr;
    std::atomic_bool m_isWillQuit;
    std::atomic_bool m_isBusy;
    std::atomic_flag m_flag = ATOMIC_FLAG_INIT;


public:
    explicit StackThread() : m_isWillQuit(false), m_isBusy(false) {}

    /* 多线程调用安全 */
    bool addAsyncTask(std::function<void(void)> &&taskFunc) {
        if (!m_flag.test_and_set()) {
            m_func = taskFunc;
            m_isBusy.exchange(true);
            return true;
        } else {
            return false;
        }
    }

    void startThread() {
        m_t = std::thread([&]() {
            while (!m_isWillQuit.load()) {
                if (m_isBusy.load()) {
                    if (m_func) {
                        m_func();
                        m_func = nullptr;
                    }
                    m_isBusy.exchange(false);
                    m_flag.clear();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    void terminate() {
        m_isWillQuit.exchange(true);
        if (m_t.joinable()) m_t.join();
    }

    ~StackThread() {
        terminate();
    }
};


class RobotManager : public QObject {
Q_OBJECT
signals:

    void loginSignal(int errorCode, bool estoped, bool poweredOn, bool servoEnabled);

    void logoutSignal(int errorCode);

    // update power_on and enable
    void updateBtSignal(int errorCode, bool poweredOn, bool servoEnabled);

    void errorSignal(int errorCode);

    void busySignal();

    void updateStatusSignal(bool isAll);

    void addInfoSignal(QString info);

private:
    StackThread m_preThread;
    StackThread m_execThread;
    StackThread m_emergencyThread;

    std::thread m_getThread;
    std::atomic_bool m_isWillGetThreadQuit;

    std::string m_currIP;
    JointValue m_tmpJVal{};

    VirtualRobot *m_pRobot = nullptr;
    RobotStatus m_status{};

    std::mutex m_getStatusMutex;

    std::mutex m_execMutex;
    ROSClient m_rosClient;
public:
    void m_startGetThread() {
        m_terminateGetThread();

        m_isWillGetThreadQuit.exchange(false);
        m_getThread = std::thread([&]() {
            RobotStatus status;
            ros::Rate rate(10);
            while (!m_isWillGetThreadQuit.load()) {
                auto res = m_pRobot->get_robot_status(&status);
                if (res == ERR_SUCC) {
                    {
                        std::lock_guard<std::mutex> lock(m_getStatusMutex);
                        memcpy(&m_status, &status, sizeof(RobotStatus));
                    }
                    emit updateStatusSignal(true);
                    m_rosClient.publish_joint_states(status.joint_position);
                } else {
                    std::cout << "get_robot_status error" << std::endl;
                }
                rate.sleep();
//                std::this_thread::sleep_for(std::chrono::milliseconds(80));
            }
        });
    }

    void m_terminateGetThread() {
        m_isWillGetThreadQuit.exchange(true);
        if (m_getThread.joinable()) m_getThread.join();
    }

public:
    RobotManager(VirtualRobot *robot,
                 ros::NodeHandle &nh,
                 const std::string &prefix) : m_isWillGetThreadQuit(false),
                                              m_rosClient(nh, prefix) {
        m_pRobot = robot;
        m_preThread.startThread();
        m_execThread.startThread();
        m_emergencyThread.startThread();
    }

    ~RobotManager() override {
        m_isWillGetThreadQuit.exchange(true);
        if (m_getThread.joinable()) m_getThread.join();
    }


    void get_robot_status(RobotStatus *status) {
        std::lock_guard<std::mutex> lock(m_getStatusMutex);
        memcpy(status, &m_status, sizeof(RobotStatus));
    }


    void get_robot_state(RobotState *state) {
        std::lock_guard<std::mutex> lock(m_getStatusMutex);
        state->estoped = m_status.emergency_stop;
        state->poweredOn = m_status.powered_on;
        state->servoEnabled = m_status.enabled;
    }

    bool is_login() {
        return m_pRobot->is_login();
    }

    void login_in(const std::string &ip) {
        m_currIP = ip;
        if (!m_preThread.addAsyncTask([&]() {
            auto res = m_pRobot->login_in(m_currIP.c_str());
            RobotStatus status;
            if (res == ERR_SUCC) {
                m_pRobot->get_robot_status(&status);
                m_startGetThread();
            }
            emit loginSignal(res, status.emergency_stop != 0, status.powered_on != 0, status.enabled != 0);
        })) {
            emit busySignal();
        }
    }

    void login_out() {
        if (!m_preThread.addAsyncTask([&]() {
            std::cout << "log out" << std::endl;
            auto res = m_pRobot->login_out();
            std::cout << "log out" << std::endl;
            if (res == ERR_SUCC) {
                m_terminateGetThread();
            }
            std::cout << "log out" << std::endl;
            emit logoutSignal(res);
        })) {
            emit busySignal();
        }
    }

    void power_on() {
        if (!m_preThread.addAsyncTask([&]() {
            auto res = m_pRobot->power_on();
            qDebug() << "m_preThread";
            emit updateBtSignal(res, res == ERR_SUCC, false);
        })) {
            emit busySignal();
        }

    }

    void power_off() {
        if (!m_preThread.addAsyncTask([&]() {
            auto res = m_pRobot->power_off();
            emit updateBtSignal(res, res != ERR_SUCC, false);
        })) {
            emit busySignal();
        }
    }

    void enable_robot() {
        if (!m_preThread.addAsyncTask([&]() {
            auto res = m_pRobot->enable_robot();
            emit updateBtSignal(res, true, res == ERR_SUCC);
        })) {
            emit busySignal();
        }
    }

    void disable_robot() {
        if (!m_preThread.addAsyncTask([&]() {
            auto res = m_pRobot->disable_robot();
            emit updateBtSignal(res, true, res != ERR_SUCC);
        })) {
            emit busySignal();
        }
    }

    void set_spin_speed(double radPerSpeed) {
        m_pRobot->set_spin_speed(radPerSpeed);
    }

    void joint_move(const JointValue *joint_pos, MoveMode move_mode) {
        if (m_execMutex.try_lock()) {
            memcpy(&m_tmpJVal, joint_pos, sizeof(JointValue));
            if (!m_execThread.addAsyncTask([&, move_mode]() {
                auto res = m_pRobot->joint_move(&m_tmpJVal, move_mode);
                emit errorSignal(res);
            })) {
                emit busySignal();
            }
            m_execMutex.unlock();
        } else {
            emit busySignal();
        }
    }

    // 阻塞
    errno_t trajectory_move(moveit_msgs::RobotTrajectory &trajectory) {
        if (!m_pRobot->is_login()) {
            emit addInfoSignal("cannot exec trajectory: robot is not login");
            return ERR_CUSTOM_NOT_LOGIN;
        }
        if (m_execMutex.try_lock()) {
            int count = 0;
            errno_t res = ERR_SUCC;
            for (const auto &p:trajectory.joint_trajectory.points) {
                JointValue jVal;
                int offset = (p.positions.size() > 6 && m_rosClient.m_prefix == "right_") ? 6 : 0;
                for (int i = 0; i < 6; ++i) {
                    jVal.jVal[i] = p.positions[i + offset];
                }
                std::cout << m_rosClient.m_prefix << "robot is moving to " << count++ << std::endl;
                res = m_pRobot->joint_move(&jVal, ABS);
                if (res != ERR_SUCC) {
                    break;
                }
            }
            m_execMutex.unlock();
            return res;
        } else {
            emit addInfoSignal("cannot exec trajectory: robot is moving");
            return ERR_CUSTOM_IS_MOVING;
        }
    }

    void set_rapid(double rapid) {
        if (!m_execThread.addAsyncTask([&, rapid]() {
            auto res = m_pRobot->set_rapidrate(rapid);
            emit errorSignal(res);
        })) {
            emit busySignal();
        }
    }

    void motion_abort() {
        if (!m_pRobot->is_login()) {
            emit addInfoSignal("cannot abort: robot is not login");
        }
        if (!m_emergencyThread.addAsyncTask([&]() {
            emit addInfoSignal("motion abort...");
            auto res = m_pRobot->motion_abort();
            emit errorSignal(res);
            emit addInfoSignal("motion abort complete");
        })) {
            emit busySignal();
        }
    }

    void collision_recover() {
        if (!m_execThread.addAsyncTask([&]() {
            auto res = m_pRobot->collision_recover();
            emit errorSignal(res);
        })) {
            emit busySignal();
        }
    }

};


#endif  // JAKA_GUI_ROBOT_MANAGER_HPP
