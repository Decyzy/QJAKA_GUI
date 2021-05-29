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

#define ERR_CUSTOM_IS_MOVING 404
#define ERR_CUSTOM_RECV_ABORT 500
#define ERR_CUSTOM_NOT_LOGIN 501


class StdErrorFactory {
public:
    std::unordered_map<errno_t, std::string> errorDescMap{};

    StdErrorFactory() {
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

        errorDescMap[ERR_CUSTOM_IS_MOVING] = "正在运动中";
        errorDescMap[ERR_CUSTOM_RECV_ABORT] = "被手动停止运动";
        errorDescMap[ERR_CUSTOM_NOT_LOGIN] = "没有登录";

    }

    // 单例
    static std::string getDesc(errno_t errorCode) {
        static StdErrorFactory instance;
        return instance.errorDescMap[errorCode];
    }
};

#endif //QJAKA_GUI_TOOLS_HPP
