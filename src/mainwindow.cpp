//
// Created by msi on 2021/5/23.
//

#include "mainwindow.h"

MainWindow::MainWindow(QMainWindow *parent) : QMainWindow(parent), spinner(4) {
    w = new QWidget(this);
    layout = new QGridLayout(w);
    mainStatusBar = new QStatusBar(this);
    w->setLayout(layout);
    setCentralWidget(w);
    setStatusBar(mainStatusBar);
    setMinimumHeight(490);
    setMaximumHeight(490);
    setMaximumWidth(1450);
    setMinimumWidth(1450);

    subWindowList.emplace_back(new SubWindow(nh, w, "left_"));
    subWindowList.emplace_back(new SubWindow(nh, w, "right_"));

    layout->addWidget(subWindowList[0], 0, 0);
    splitLine = new QFrame(w);
    splitLine->setObjectName(QString::fromUtf8("line"));
    splitLine->setFrameShape(QFrame::HLine);
    splitLine->setFrameShadow(QFrame::Sunken);

    layout->addWidget(splitLine, 1, 0);
    layout->addWidget(subWindowList[1], 2, 0);

    mainStatusBar->addPermanentWidget(&currentTimeLabel);
    connect(&timer, &QTimer::timeout, this, &MainWindow::onTimeout);
    timer.start(500);

    m_doSrv = nh.advertiseService<qjaka_gui::DigitalOutputService::Request, qjaka_gui::DigitalOutputService::Response>(
            "digital_output_srv",
            [&](qjaka_gui::DigitalOutputService::Request &req,
                qjaka_gui::DigitalOutputService::Response &resp) -> bool {
                std::cout << "recv do request" << std::endl;
                RobotManager *rm = subWindowList[1]->rm;
                for (int i = 0; i < req.index.size(); ++i) {
                    rm->set_do_sync(req.index[i], req.enable[i] > 0);
                }
                resp.success = true;
                resp.desc = "";
                return true;
            });


    m_trajectorySrv = nh.advertiseService<qjaka_gui::DualRobotJointMoveService::Request, qjaka_gui::DualRobotJointMoveService::Response>(
            "dual_trajectory_srv",
            [&](qjaka_gui::DualRobotJointMoveService::Request &req,
                qjaka_gui::DualRobotJointMoveService::Response &resp) -> bool {
                std::cout << "recv dual trajectory" << std::endl;
                // exec
                std::mutex respMutex;
                std::thread t0;
                std::thread t1;
                resp.success = true;
                t0 = std::thread([&]() {
                    if (!req.enable_left) {
                        return;
                    }
                    errno_t res;
                    res = subWindowList[0]->rm->trajectory_move_v2(req.left_joint_values, req.step_num);
                    if (res != ERR_SUCC) {
                        subWindowList[1]->rm->motion_abort();
                        std::lock_guard<std::mutex> lock(respMutex);
                        resp.success = false;
                        resp.left_desc += "left:" + ErrorDescFactory::build()->getErrorDesc(res);
                    }
                });

                t1 = std::thread([&]() {
                    if (!req.enable_right) {
                        return;
                    }
                    errno_t res;
                    res = subWindowList[1]->rm->trajectory_move_v2(req.right_joint_values, req.step_num);
                    if (res != ERR_SUCC) {
                        subWindowList[0]->rm->motion_abort();
                        std::lock_guard<std::mutex> lock(respMutex);
                        resp.success = false;
                        resp.right_desc += "right:" + ErrorDescFactory::build()->getErrorDesc(res);
                    }
                });

                if (t0.joinable()) t0.join();
                if (t1.joinable()) t1.join();

                // finish
                std::cout << "exec trajectory finished" << std::endl;
                return true;
            });
    spinner.start();
}


void MainWindow::onTimeout() {
    currentTimeLabel.setText(QDateTime::currentDateTime().toString("hh:mm:ss"));
}


MainWindow::~MainWindow() {
    for (auto &p: subWindowList) {
        delete p;
    }
    delete splitLine;
    delete layout;
    delete w;
//    std::cout << "ros::waitForShutdown()" << std::endl;
//    ros::waitForShutdown();
//    std::cout << "all quit" << std::endl;
}
