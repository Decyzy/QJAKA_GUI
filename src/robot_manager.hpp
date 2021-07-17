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
#include <condition_variable>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <tf2_ros/transform_broadcaster.h>
#include "qjaka_gui/JointMoveService.h"
#include "qjaka_gui/GraspService.h"

#include "robot.hpp"

class ROSPublisher {
private:
    ros::NodeHandle &m_nh;
    sensor_msgs::JointState m_jointStateMsg;
    geometry_msgs::TransformStamped m_tfMsg;

    ros::Publisher m_jointStatePub;
    tf2_ros::TransformBroadcaster m_tfPub;

public:
    const std::string m_prefix;

    explicit ROSPublisher(ros::NodeHandle &nh, std::string prefix) : m_nh(nh), m_prefix(std::move(prefix)) {
        m_jointStatePub = m_nh.advertise<sensor_msgs::JointState>("jaka_joint_states", 5);

        m_jointStateMsg = sensor_msgs::JointState();
        m_jointStateMsg.name.resize(6);
        m_jointStateMsg.position.resize(6);
        for (int i = 0; i < 6; ++i) {
            m_jointStateMsg.name[i] = m_prefix + "joint_" + std::to_string(i + 1);
            m_jointStateMsg.position[i] = 0.0;
        }

        m_tfMsg.header.frame_id = m_prefix + "base_link";
        m_tfMsg.child_frame_id = m_prefix + "real_ee_link";
    }

    void publish_joint_states(const double states[6]) {
        m_jointStateMsg.header.stamp = ros::Time::now();
        for (int i = 0; i < 6; ++i) {
            m_jointStateMsg.position[i] = states[i];
        }
        m_jointStatePub.publish(m_jointStateMsg);
    }

    void publish_ee_tf(const double *pose, tf2::Quaternion &qua) {
        m_tfMsg.header.stamp = ros::Time::now();
        m_tfMsg.transform.translation.x = pose[0] / 1000.0;
        m_tfMsg.transform.translation.y = pose[1] / 1000.0;
        m_tfMsg.transform.translation.z = pose[2] / 1000.0;
        qua.setRPY(pose[3], pose[4], pose[5]);
        tf2::convert(qua, m_tfMsg.transform.rotation);
        m_tfPub.sendTransform(m_tfMsg);
    }
};


class StackThread {
private:
    std::thread m_t;
    std::function<void(void)> m_taskFunc = nullptr;
    std::atomic_bool m_willQuit;
    std::atomic_bool m_isReady;
    std::atomic_flag m_flag = ATOMIC_FLAG_INIT;

    std::condition_variable m_cv;
    std::mutex m_mutex;

public:
    explicit StackThread() : m_willQuit(false), m_isReady(false) {}

    /* 多线程调用安全 */
    bool addAsyncTask(std::function<void(void)> &&taskFunc) {
        if (!m_flag.test_and_set()) {
            m_taskFunc = taskFunc;
            m_isReady.exchange(true);
            m_cv.notify_all();
            return true;
        } else {
            return false;
        }
    }

    void startThread() {
        m_t = std::thread([&]() {
            while (!m_willQuit.load()) {
                std::unique_lock<std::mutex> lk(m_mutex);
                m_cv.wait(lk, [&] { return m_isReady.load() || m_willQuit.load(); });
                if (m_taskFunc) {
                    m_taskFunc();
                    m_taskFunc = nullptr;
                }
                m_isReady.exchange(false);
                m_flag.clear();
                lk.unlock();
            }
        });
    }

    void terminate() {
        m_willQuit.exchange(true);
        m_cv.notify_all();
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
    void updateBtSignal(QString name, int errorCode, bool poweredOn, bool servoEnabled);

    void errorSignal(QString name, int errorCode);

    void busySignal(QString name);

    void updateStatusSignal(bool isAll);

    void addInfoSignal(QString info);

    void updateCollisionLevel(int);

private:
    StackThread m_preThread;
    StackThread m_execThread;
    StackThread m_emergencyThread;

    std::thread m_getThread;
    std::atomic_bool m_getThreadWillQuit;

    std::string m_currIP;
    JointValue m_tmpJVal{};

    VirtualRobot *m_pRobot = nullptr;
    RobotStatus m_status{};
    tf2::Quaternion m_qua;

    std::mutex m_getStatusMutex;

    std::mutex m_execMutex;
    ROSPublisher m_rosPublisher;

    std::string m_prefix;

public:
    void m_startGetThread() {
        m_terminateGetThread();

        m_getThreadWillQuit.exchange(false);
        m_getThread = std::thread([&]() {
            RobotStatus status{};
            ros::Rate rate(10);
            while (!m_getThreadWillQuit.load()) {
                errno_t res = m_pRobot->get_robot_status(&status);
                if (res == ERR_SUCC) {
                    {
                        std::lock_guard<std::mutex> lock(m_getStatusMutex);
                        memcpy(&m_status, &status, sizeof(RobotStatus));
                        m_rosPublisher.publish_ee_tf(status.cartesiantran_position, m_qua);
                    }
                    emit updateStatusSignal(true);
                    m_rosPublisher.publish_joint_states(status.joint_position);
                } else {
                    std::cout << "get_robot_status error" << std::endl;
                }
                //
                rate.sleep();
                // std::this_thread::sleep_for(std::chrono::milliseconds(80));
            }
        });
    }

    void m_terminateGetThread() {
        m_getThreadWillQuit.exchange(true);
        if (m_getThread.joinable()) m_getThread.join();
    }

public:
    RobotManager(VirtualRobot *robot,
                 ros::NodeHandle &nh,
                 const std::string &prefix) : m_getThreadWillQuit(false),
                                              m_rosPublisher(nh, prefix),
                                              m_prefix(prefix) {
        m_pRobot = robot;
        m_pRobot->set_prefix(prefix);
        m_pRobot->set_initial_status();
        m_preThread.startThread();
        m_execThread.startThread();
        m_emergencyThread.startThread();
    }

    ~RobotManager() override {
        m_getThreadWillQuit.exchange(true);
        if (m_getThread.joinable()) m_getThread.join();
    }

    bool is_own_virtual() {
        return m_pRobot->is_virtual();
    }

    void get_robot_status(RobotStatus *status, tf2::Quaternion &qua) {
        std::lock_guard<std::mutex> lock(m_getStatusMutex);
        memcpy(status, &m_status, sizeof(RobotStatus));
        qua.setX(m_qua.x());
        qua.setY(m_qua.y());
        qua.setZ(m_qua.z());
        qua.setW(m_qua.w());
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
                int collisionLevel = -1;
                errno_t res2 = m_pRobot->get_collision_level(&collisionLevel);
                if (res2 == ERR_SUCC) emit updateCollisionLevel(collisionLevel);
                else emit errorSignal("get_collision_level", res2);
                m_startGetThread();
            }
            emit loginSignal(res, status.emergency_stop != 0, status.powered_on != 0, status.enabled != 0);
        })) {
            emit busySignal("login_in");
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
            emit busySignal("login_out");
        }
    }

    void power_on() {
        if (!m_preThread.addAsyncTask([&]() {
            auto res = m_pRobot->power_on();
            qDebug() << "m_preThread";
            emit updateBtSignal("power_on", res, res == ERR_SUCC, false);
        })) {
            emit busySignal("power_on");
        }
    }

    void power_off() {
        if (!m_preThread.addAsyncTask([&]() {
            auto res = m_pRobot->power_off();
            emit updateBtSignal("power_off", res, res != ERR_SUCC, false);
        })) {
            emit busySignal("power_off");
        }
    }

    void enable_robot() {
        if (!m_preThread.addAsyncTask([&]() {
            auto res = m_pRobot->enable_robot();
            emit updateBtSignal("enable_robot", res, true, res == ERR_SUCC);
        })) {
            emit busySignal("enable_robot");
        }
    }

    void disable_robot() {
        if (!m_preThread.addAsyncTask([&]() {
            auto res = m_pRobot->disable_robot();
            emit updateBtSignal("disable_robot", res, true, res != ERR_SUCC);
        })) {
            emit busySignal("disable_robot");
        }
    }

    void set_spin_vel_and_acc(double radPerSecond, double accPerSecondPerSecond) {
        m_pRobot->set_spin_vel_and_acc(radPerSecond, accPerSecondPerSecond);
    }

    void joint_move(const JointValue *joint_pos, MoveMode move_mode) {
        if (m_execMutex.try_lock()) {
            memcpy(&m_tmpJVal, joint_pos, sizeof(JointValue));
            if (!m_execThread.addAsyncTask([&, move_mode]() {
                auto res = m_pRobot->joint_move(&m_tmpJVal, move_mode);
                emit errorSignal("joint_move", res);
            })) {
                emit busySignal("joint_move");
            }
            m_execMutex.unlock();
        } else {
            emit busySignal("joint_move");
        }
    }

    errno_t joint_move_sync(const JointValue *joint_pos, MoveMode move_mode) {
        return m_pRobot->joint_move(joint_pos, move_mode);
    }
//
//    // 阻塞
//    errno_t trajectory_move(moveit_msgs::RobotTrajectory &trajectory) {
//        if (!m_pRobot->is_login()) {
//            emit addInfoSignal("cannot exec trajectory: robot is not login");
//            return ERR_CUSTOM_NOT_LOGIN;
//        }
//        if (m_execMutex.try_lock()) {
//            int count = 0;
//            errno_t res = ERR_SUCC;
//            int offset = (trajectory.joint_trajectory.points[0].positions.size() > 6 &&
//                          m_rosPublisher.m_prefix == "right_") ? 6 : 0;
//            for (const auto &p:trajectory.joint_trajectory.points) {
//                JointValue jVal;
//                for (int i = 0; i < 6; ++i) {
//                    jVal.jVal[i] = p.positions[i + offset];
//                }
//                std::cout << m_rosPublisher.m_prefix << "robot is moving to " << count++ << std::endl;
//                res = m_pRobot->joint_move(&jVal, ABS);
//                if (res != ERR_SUCC) {
//                    break;
//                }
//            }
//            m_execMutex.unlock();
//            return res;
//        } else {
//            emit addInfoSignal("cannot exec trajectory: robot is moving");
//            return ERR_CUSTOM_IS_MOVING;
//        }
//    }

    // 阻塞
    /**
     * 滤波器参数都是越大越精确
     * @param jVals
     * @param step_num 1
     * @param max_buf 20
     * @param kp 0.1, 0.05 ~ 0.3
     * @param kv 0.2, 0.1 ~ 0.5
     * @param ka 0.6, 0.5 ~ 0.9
     * @return
     */
    errno_t trajectory_move_v2(std::vector<double> &jVals, const unsigned int step_num = 1,
                               int max_buf = 20, double kp = 0.1, double kv = 0.2, double ka = 0.6) {

        if (!m_pRobot->is_login()) {
            emit addInfoSignal("cannot exec trajectory: robot is not login");
            return ERR_CUSTOM_NOT_LOGIN;
        }
        if (m_execMutex.try_lock()) {
            // check trajectory
            if (!is_own_virtual()) {
                std::lock_guard<std::mutex> lock(m_getStatusMutex);
                for (int i = 0; i < 5; ++i) {
                    if (std::fabs(m_status.joint_position[i] - jVals[i]) > 0.1) {
                        m_execMutex.unlock();
                        return ERR_CUSTOM_INVALID_TRAJECTORY;
                    }
                }
            }
            double j6_val = m_status.joint_position[5];
            // move
            errno_t res = ERR_SUCC;
            res = m_pRobot->servo_move_use_joint_MMF(max_buf, kp, kv, ka);
            emit errorSignal("servo_move_use_joint_MMF", res);
            res = m_pRobot->servo_move_enable(true);
            if (res == ERR_SUCC) {
                std::cout << "start move" << std::endl;
                int i = 0;
                JointValue _jVal;
                int batch = is_own_virtual() ? 1 : 100;  // 每轮跑 batch 组
                double scale = is_own_virtual() ? 1.0 : 0.8;
                ros::Rate rate(1000.0 / (8.0 * batch * scale * double(step_num)));
                int count = 0;
                while (i < jVals.size()) {
                    for (int j = 0; j < 5; ++j) {
                        _jVal.jVal[j] = jVals[i + j];
                    }
                    _jVal.jVal[5] = j6_val;
                    i += 6;
                    res = m_pRobot->servo_j(&_jVal, ABS, step_num);
                    if (res != ERR_SUCC) {
                        for (double j : _jVal.jVal)
                            std::cout << j << ", ";
                        std::cout << std::endl;
                        emit errorSignal("servo_j", res);
                        break;
                    }
                    if (++count == batch) {
                        rate.sleep();
                        count = 0;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            } else {
                emit errorSignal("servo_move_enable(true)", res);
            }
            if (res == ERR_SUCC) {
                res = m_pRobot->servo_move_enable(false);
                emit errorSignal("servo_move_enable(false)", res);
            }

//            res = m_pRobot->motion_abort();
//            emit errorSignal("motion_abort", res);
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
            emit errorSignal("set_rapidrate", res);
        })) {
            emit busySignal("set_rapid");
        }
    }

    void motion_abort() {
        if (!m_pRobot->is_login()) {
            emit addInfoSignal("cannot abort: robot is not login");
        }
        if (!m_emergencyThread.addAsyncTask([&]() {
            emit addInfoSignal("motion abort...");
            auto res = m_pRobot->motion_abort();
            emit errorSignal("motion_abort", res);
            emit addInfoSignal("motion abort complete");
        })) {
            emit busySignal("motion_abort");
        }
    }

    void collision_recover() {
        if (!m_emergencyThread.addAsyncTask([&]() {
            auto res = m_pRobot->collision_recover();
            emit errorSignal("collision_recover", res);
        })) {
            emit busySignal("collision_recover");
        }
    }

    void set_collision_level(const int level) {
        if (m_execMutex.try_lock()) {
            if (!m_execThread.addAsyncTask([&, level]() {
                auto res = m_pRobot->set_collision_level(level);
                emit errorSignal("set_collision_level", res);
            })) {
                emit busySignal("set_collision_level");
            }
            m_execMutex.unlock();
        } else {
            emit busySignal("set_collision_level");
        }
    }

    void get_collision_level() {
        if (!m_preThread.addAsyncTask([&]() {
            int collisionLevel = -1;
            errno_t res2 = m_pRobot->get_collision_level(&collisionLevel);
            if (res2 == ERR_SUCC) emit updateCollisionLevel(collisionLevel);
            else emit errorSignal("get_collision_level", res2);
        })) {
            emit busySignal("get_collision_level");
        }
    }

    void set_do(int index, bool enable) {
        if (!m_emergencyThread.addAsyncTask([&, index, enable]() {
            auto res = m_pRobot->set_do(index, enable);
            emit errorSignal("set_do", res);
        })) {
            emit busySignal("set_ee_open");
        }
    }
};


#endif  // JAKA_GUI_ROBOT_MANAGER_HPP
