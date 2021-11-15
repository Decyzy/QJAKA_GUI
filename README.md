# QJAKA_GUI

节卡机械臂图形界面 + ROS

![gui_snap](doc/gui_snap.png)

## 依赖

- ROS

- jaka sdk （ 本仓库doc下）：

  - 修改`CmakeLists.txt`中的`set(Jaka_DIR "/home/msi/Opt/jaka_sdk")`（35行附近），修改为你的路径

  - ```bash
    # sdk文件夹结构示例:
    /home/msi/Opt/jaka_sdk
    ├── include
    │   ├── JAKAZuRobot.h
    │   ├── jkerr.h
    │   └── jktypes.h
    ├── JakaConfig.cmake
    └── lib
        ├── libjakaAPI.so
        └── libz.a
    ```

- robot_models：本仓库doc下zip压缩包，包含zu7 2.5版本和1.0版本urdf

  - `zu7/urdf/jaka_macro.xacro`：添加了macro `prefix`，用以区分左右机械臂；
  - `jaka_zu7_v2/urdf/jaka_zu7_v2.xacro`：添加了macro `prefix`，用以区分左右机械臂；
  - `zu7/urdf/jaka.urdf.xacro`：官方文件，单机械臂；
  - `qjaka_gui/urdf/jaka_dual.xacro`：双机械臂例子

## 测试

```bash
roslaunch qjaka_gui test_gui.launch
```

## 程序设计

### 机器人类 `VirtualRobot` 和 `RealRobot`

使用多态特性，封装机械臂访问和控制的统一阻塞接口。

- `VirtualRobot`：虚拟机械臂，接口为虚函数，阻塞接口；

- `RealRobot`：继承自`VirtualRobot`，代理节卡sdk接口；

### 机器人管理类 `RobotManager`

将机器人类的阻塞接口转为非阻塞接口，供GUI线程调用，同时emit QT信号来刷新界面。

其中包含了四个线程来执行机器人类中的阻塞函数，继承自`StackThread`类的线程接口只允许串行执行任务，例如一次`joint_move`任务被委托到`m_execThread`线程，在没有执行完毕前，`m_execThread`不再接受其他任务。即线程内串行，线程间并行：

- `m_preThread`：执行登录、上电、使能；
- `getThread`：成功登录后，以固定频率获取机器人全部状态(`get_robot_status`)
- `m_execThread`；执行关节移动等改变机器人状态的命令；
- `m_emergencyThread`：执行紧急函数，包括终止运动，碰撞恢复；

### 关于`ROS`

`prefix` 硬编码为 `left_` 或 `right_`

#### Param

- `left_ip`，`right_ip`：设置机械臂的默认 IP

#### Topic

- `jaka_joint_states`
  - Type：`sensor_msgs::JointState`
  - Desc：在login成功后，随着`getThread`的执行，机械臂关节信息也一并以固定频率发布
- TF：`<prefix>gripper_base_link`
  - `<prefix>gripper_base_link`在真机上的位姿。虚拟机器人上二者重合，真机上不重合（因机器人模型与真机的DH参数不同）

#### Service

- `<prefix>trajectory_srv`

  - Type：见 `srv/JointMoveService.srv`
  - Desc：单机械臂运动。其中 `joint_values` 为 6 关节数据拍平后一维数组，即每6个数据代表一个关节坐标。内部调用机器人的伺服运动接口。

- `dual_trajectory_srv`

  - Type：见 `srv/DualRobotJointMoveService.srv`
  - Desc：双机械臂同步运动。

- `digital_output_srv`

  - Type：见 `srv/DigitalOutputService.srv`
  - Desc：设置机械臂控制柜的数字输出。硬编码为右机械臂的控制柜。

- `<prefix>grasp_srv`

  - Type：见 `srv/GraspService.srv`
  - Desc：执行抓取动作，末端关节按指定参数旋转

  

