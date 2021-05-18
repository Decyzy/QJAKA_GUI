//
// Created by msi on 2021/5/18.
//

#ifndef QJAKA_GUI_ROBOT_HPP
#define QJAKA_GUI_ROBOT_HPP

#include <QObject>

#include <JAKAZuRobot.h>
#include <jkerr.h>
#include <jktypes.h>

#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <functional>
#include <iostream>

#define ERR_BUSY 404


class VirtualRobot {
private:
    std::atomic_bool m_isLogin;
    RobotStatus m_status;

    void m_sleepMilliseconds(long long milliseconds=500) {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));

    }

public:
    VirtualRobot() : m_isLogin(false), m_status({}) {}

    bool is_login() { return m_isLogin.load(); }

    errno_t login_in(const char *ip) {
        m_sleepMilliseconds();

        m_isLogin.exchange(true);
        m_status.is_socket_connect = 1;
        return ERR_SUCC;
    }

    errno_t login_out() {
        m_sleepMilliseconds();

        m_isLogin.exchange(false);
        return ERR_SUCC;
    }

    errno_t power_on() {
        m_sleepMilliseconds();

        m_status.powered_on = 1;
        return ERR_SUCC;
    }

    errno_t power_off() {
        m_sleepMilliseconds();

        m_status.powered_on = 0;
        return ERR_SUCC;
    }

    errno_t enable_robot() {
        m_sleepMilliseconds();

        m_status.enabled = 1;
        return ERR_SUCC;
    }

    errno_t disable_robot() {
        m_sleepMilliseconds();

        m_status.enabled = 0;
        return ERR_SUCC;
    }

    errno_t motion_abort() { return ERR_SUCC; }

    errno_t collision_recover() { return ERR_SUCC; }

    errno_t get_joint_position(JointValue *joint_position) { return ERR_SUCC; }

    errno_t get_robot_state(RobotState *state) {
        state->poweredOn = m_status.powered_on;
        state->servoEnabled = m_status.enabled;
        return ERR_SUCC;
    }

    errno_t get_robot_status(RobotStatus *status) {
        memcpy(status, &m_status, sizeof(RobotStatus));
        return ERR_SUCC;
    }

};


class RobotManager : public QObject {
Q_OBJECT
private:
    std::thread m_getThread;
    std::thread m_setThread;
    std::atomic_bool m_hasNormalTask;
    std::function<void(void)> m_normalTask;

    VirtualRobot *m_pRobot;
    VirtualRobot m_virtualRobot;
    std::atomic_bool m_isWillTerminate;
    std::atomic_bool m_isWillGetThreadTerminate;
    std::mutex m_mutex;

    RobotStatus m_status;
    std::unordered_map<errno_t, std::string> m_errorDescMap;

    void m_addAsyncTask(std::function<void(void)> &&func) {
        if (m_hasNormalTask.load()) {
            emit busySignal();
        } else {
            m_normalTask = func;
            m_hasNormalTask.exchange(true);
        }
    }

signals:

    void loginSignal(int errorCode, bool estoped, bool poweredOn, bool servoEnabled);

    void logoutSignal(int errorCode);

    // update power_on and enable
    void updateBtSignal(int errorCode, bool poweredOn, bool servoEnabled);

    void errorSignal(int errorCode);

    void busySignal();

    void updateStatusSignal(bool isAll);

public:
    RobotManager() : m_hasNormalTask(false), m_isWillTerminate(false), m_isWillGetThreadTerminate(false), m_status({}) {

        m_errorDescMap[ERR_SUCC] = "调用成功";
        m_errorDescMap[ERR_FUCTION_CALL_ERROR] = "异常调用，调用接口异常，控制器不支持";
        m_errorDescMap[ERR_INVALID_HANDLER] = "无效的控制句柄";
        m_errorDescMap[ERR_INVALID_PARAMETER] = "无效的参数";
        m_errorDescMap[ERR_COMMUNICATION_ERR] = "通信连接错误";
        m_errorDescMap[ERR_KINE_INVERSE_ERR] = "逆解失败";
        m_errorDescMap[ERR_EMERGENCY_PRESSED] = "急停开关被按下";
        m_errorDescMap[ERR_NOT_POWERED] = "机器人未上电";
        m_errorDescMap[ERR_NOT_ENABLED] = "机器人未使能";
        m_errorDescMap[ERR_DISABLE_SERVOMODE] = "机器人没有进入servo模式";
        m_errorDescMap[ERR_NOT_OFF_ENABLE] = "机器人没有关闭使能";
        m_errorDescMap[ERR_PROGRAM_IS_RUNNING] = "程序正在运行，不允许操作";
        m_errorDescMap[ERR_CANNOT_OPEN_FILE] = "无法打开文件，文件不存在";
        m_errorDescMap[ERR_MOTION_ABNORMAL] = "运动过程中发生异常";

        m_pRobot = &m_virtualRobot;

        m_setThread = std::thread([&]() {
            std::cout << "m_setThread start" << std::endl;
            while (!m_isWillTerminate.load()) {
                if (m_hasNormalTask.load()) {
                    m_normalTask();
                    m_hasNormalTask.exchange(false);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            std::cout << "m_setThread quit" << std::endl;
        });
    }

    ~RobotManager() override {
        m_isWillTerminate.exchange(true);
        if (m_getThread.joinable()) m_getThread.join();
        if (m_setThread.joinable()) m_setThread.join();
    }

    void start_get_thread() {
        m_isWillTerminate.exchange(true);
        if (m_getThread.joinable()) m_getThread.join();

        m_isWillTerminate.exchange(false);
        m_getThread = std::thread([&]() {
            std::cout << "m_getThread start" << std::endl;
            int count = 0;
            while (!m_isWillGetThreadTerminate.load() && !m_isWillTerminate.load()) {
                if (count % 3 == 0) {
                    RobotStatus status = {};
                    m_pRobot->get_robot_status(&status);
                    {
                        std::lock_guard<std::mutex> lock(m_mutex);
                        memcpy(&m_status, &status, sizeof(RobotStatus));
                    }
                    emit updateStatusSignal(true);
                } else {
                    JointValue jointValue;
                    m_pRobot->get_joint_position(&jointValue);
                    {
                        std::lock_guard<std::mutex> lock(m_mutex);
                        memcpy(m_status.joint_position, jointValue.jVal, sizeof(jointValue.jVal));
                    }
                    emit updateStatusSignal(false);
                }
                if (++count > 12) {
                    count = 0;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(60));
            }
            std::cout << "m_getThread quit" << std::endl;
        });
    }

    void stop_get_thread() {
        m_isWillGetThreadTerminate.exchange(true);
        if (m_getThread.joinable()) m_getThread.join();
        m_isWillGetThreadTerminate.exchange(false);
    }

    void login_in(const char *ip) {
        char tmp[1024];
        strcpy(tmp, ip);
        m_addAsyncTask([&, tmp]() {
            auto res = m_pRobot->login_in(tmp);
            RobotState state = {};
            if (res == ERR_SUCC) {
                m_pRobot->get_robot_state(&state);
            }
            emit loginSignal(res, state.estoped != 0, state.poweredOn != 0, state.servoEnabled != 0);
        });
    }

    void login_out() {
        m_addAsyncTask([&]() {
            auto res = m_pRobot->login_out();
            emit logoutSignal(res);
        });
    }

    void power_on() {
        m_addAsyncTask([&]() {
            auto res = m_pRobot->power_on();
            emit updateBtSignal(res, res == ERR_SUCC, false);
        });

    }

    void power_off() {
        m_addAsyncTask([&]() {
            auto res = m_pRobot->power_off();
            emit updateBtSignal(res, res != ERR_SUCC, false);
        });
    }

    void enable_robot() {
        m_addAsyncTask([&]() {
            auto res = m_pRobot->enable_robot();
            emit updateBtSignal(res, true, res == ERR_SUCC);
        });
    }

    void disable_robot() {
        m_addAsyncTask([&]() {
            auto res = m_pRobot->disable_robot();
            emit updateBtSignal(res, true, res != ERR_SUCC);
        });
    }

    bool is_login() {
        return m_pRobot->is_login();
    }

    std::string get_error_desc(errno_t errorCode) {
        if (m_errorDescMap.count(errorCode) == 0)
            return "未知错误";
        else return m_errorDescMap[errorCode];
    }

    void get_all_status(RobotStatus *status) {
        std::lock_guard<std::mutex> lock(m_mutex);
        memcpy(status, &m_status, sizeof(RobotStatus));
    }

    void get_robot_state(RobotState *state) {
        std::lock_guard<std::mutex> lock(m_mutex);
        state->poweredOn = m_status.powered_on;
        state->servoEnabled = m_status.enabled;
        state->estoped = m_status.emergency_stop;
    }
};

#endif  // QJAKA_GUI_ROBOT_HPP
