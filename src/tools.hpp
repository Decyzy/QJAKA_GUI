//
// Created by msi on 2021/5/29.
//

#ifndef QJAKA_GUI_TOOLS_HPP
#define QJAKA_GUI_TOOLS_HPP

#include <JAKAZuRobot.h>
#include <jkerr.h>
#include <jktypes.h>
#include <iostream>
#include <bprinter/table_printer.h>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#define ERR_CUSTOM_IS_MOVING 404
#define ERR_CUSTOM_RECV_ABORT 500
#define ERR_CUSTOM_NOT_LOGIN 501
#define ERR_CUSTOM_INVALID_TRAJECTORY 503

class ErrorDescFactory {
public:
    std::unordered_map<int, std::string> errorDescMap{};

    explicit ErrorDescFactory() {
        errorDescMap[ERR_SUCC] = "调用成功";
        errorDescMap[ERR_FUCTION_CALL_ERROR] = "异常调用，调用接口异常，控制器不支持";
        errorDescMap[ERR_INVALID_HANDLER] = "无效的控制句柄";
        errorDescMap[ERR_INVALID_PARAMETER] = "无效的参数";
        errorDescMap[ERR_COMMUNICATION_ERR] = "通信连接错误";
        errorDescMap[ERR_KINE_INVERSE_ERR] = "逆解失败";
        errorDescMap[ERR_EMERGENCY_PRESSED] = "急停开关被按下";
        errorDescMap[ERR_NOT_POWERED] = "机器人未上电";
        errorDescMap[ERR_NOT_ENABLED] = "机器人未使能";
        errorDescMap[ERR_DISABLE_SERVOMODE] = "机器人没有进入servo模式";
        errorDescMap[ERR_NOT_OFF_ENABLE] = "机器人没有关闭使能";
        errorDescMap[ERR_PROGRAM_IS_RUNNING] = "程序正在运行，不允许操作";
        errorDescMap[ERR_CANNOT_OPEN_FILE] = "无法打开文件，文件不存在";
        errorDescMap[ERR_MOTION_ABNORMAL] = "运动过程中发生异常";
        /* custom error code */
        errorDescMap[ERR_CUSTOM_IS_MOVING] = "正在运动中";
        errorDescMap[ERR_CUSTOM_RECV_ABORT] = "被手动停止运动";
        errorDescMap[ERR_CUSTOM_NOT_LOGIN] = "没有登录";
        errorDescMap[ERR_CUSTOM_INVALID_TRAJECTORY] = "轨迹无效, 跳跃的起始点";

    }

    static ErrorDescFactory* build() {
        static ErrorDescFactory instance;
        return &instance;
    }

    // 单例
    std::string getErrorDesc(errno_t errorCode) {
        if (!errorDescMap.count(errorCode)) return "未知错误!手册中没写, 错误码:" + std::to_string(errorCode);
        else return errorDescMap[errorCode];
    }
};


//
//class PoseTransform {
//public:
//    // rad, mm
//    static void rpyPose2quaPose(const double rpyPose[6], geometry_msgs::Pose &quaPose) {
//        quaPose.position.x = rpyPose[0] / 1000.0;
//        quaPose.position.y = rpyPose[1] / 1000.0;
//        quaPose.position.z = rpyPose[2] / 1000.0;
//
//        Eigen::Vector3d eulerAngle(rpyPose[3], rpyPose[4], rpyPose[5]);
//        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
//        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
//        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
//        Eigen::Quaterniond qua = yawAngle * pitchAngle * rollAngle;
//
//        quaPose.orientation.x = qua.x();
//        quaPose.orientation.y = qua.y();
//        quaPose.orientation.z = qua.z();
//        quaPose.orientation.w = qua.w();
//        static int a = 0;
//    }
//
//    static void qua2rpy(const geometry_msgs::Quaternion &qua, double rpy[3]) {
//        Eigen::Quaterniond quaternion(qua.w, qua.x, qua.y, qua.z);
//        Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2, 1, 0);
//        rpy[0] = eulerAngle(2);
//        rpy[1] = eulerAngle(1);
//        rpy[2] = eulerAngle(0);
//    }
//};

#endif //QJAKA_GUI_TOOLS_HPP
