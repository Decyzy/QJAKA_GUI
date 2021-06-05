//
// Created by msi on 2021/5/23.
//

#include "mainwindow.h"

MainWindow::MainWindow(QMainWindow *parent) : QMainWindow(parent), spinner(1) {
    w = new QWidget(this);
    layout = new QGridLayout(w);
    mainStatusBar = new QStatusBar(this);
    w->setLayout(layout);
    setCentralWidget(w);
    setStatusBar(mainStatusBar);

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

    // todo: 对单个机械臂提供单独服务
    m_trajectorySrv = nh.advertiseService<qjaka_gui::JointMoveService::Request, qjaka_gui::JointMoveService::Response>(
            "jaka_trajectory_srv",
            [&](qjaka_gui::JointMoveService::Request &req, qjaka_gui::JointMoveService::Response &resp) -> bool {
                // init
                resp.success = true;
                // print info
                std::cout << "recv trajectory" << std::endl;
                bprinter::TablePrinter tp(&std::cout);
                tp.AddColumn("Joint Name", 14);
                for (int i = 0; i < req.trajectory.joint_trajectory.points.size(); ++i) {
                    tp.AddColumn(std::to_string(i), 8);
                }
                tp.PrintHeader();
                for (int i = 0; i < req.trajectory.joint_trajectory.joint_names.size(); ++i) {
                    tp << req.trajectory.joint_trajectory.joint_names[i];
                    for (const auto &p:req.trajectory.joint_trajectory.points) {
                        tp << int(p.positions[i] / M_PI * 180.0 * 10) / 10.0;
                    }
                }
                tp.PrintFooter();
                // exec
                std::mutex respMutex;
                std::thread t0;
                std::thread t1;
                if (req.prefix.data == "left_" || req.trajectory.joint_trajectory.joint_names.size() > 6) {
                    t0 = std::thread([&]() {
                        auto res = subWindowList[0]->rm->trajectory_move(req.trajectory);
                        if (res != ERR_SUCC) {
                            subWindowList[1]->rm->motion_abort();
                            std::lock_guard<std::mutex> lock(respMutex);
                            resp.success = false;
                            resp.left_desc.data = DescFactory::getErrorDesc(res);
                        }
                    });
                } else {
                    resp.left_desc.data = "不关我事";
                }
                if (req.prefix.data == "right_" || req.trajectory.joint_trajectory.joint_names.size() > 6) {
                    t1 = std::thread([&]() {
                        auto res = subWindowList[1]->rm->trajectory_move(req.trajectory);
                        if (res != ERR_SUCC) {
                            subWindowList[0]->rm->motion_abort();
                            std::lock_guard<std::mutex> lock(respMutex);
                            resp.success = false;
                            resp.right_desc.data = DescFactory::getErrorDesc(res);
                        }
                    });
                } else {
                    resp.right_desc.data = "不关我事";
                }
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
    for (auto &p:subWindowList) {
        delete p;
    }
    delete splitLine;
    delete layout;
    delete w;
//    std::cout << "ros::waitForShutdown()" << std::endl;
//    ros::waitForShutdown();
//    std::cout << "all quit" << std::endl;
}
