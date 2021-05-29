//
// Created by msi on 2021/5/23.
//

#ifndef JAKA_GUI_MAINWINDOW_H
#define JAKA_GUI_MAINWINDOW_H

#include "subwindow.h"
#include <QStatusBar>
#include "qjaka_gui/JointMoveService.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <iomanip>

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(QMainWindow *parent = nullptr);

    ~MainWindow() override;

private:
    QGridLayout *layout;
    QWidget *w;
    QStatusBar *mainStatusBar;
    QTimer timer;
    QLabel currentTimeLabel;

    std::vector<SubWindow*> subWindowList;
    QFrame *splitLine;

    ros::NodeHandle nh;
    ros::ServiceServer m_trajectorySrv;
    ros::AsyncSpinner spinner;

    void onTimeout();
};


#endif //JAKA_GUI_MAINWINDOW_H
