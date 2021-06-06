#include "subwindow.h"

#include "./ui_subwindow.h"


QColor primaryGreenColor = QColor::fromRgb(0x39, 0xa3, 0x0e);
QColor primaryRedColor = QColor::fromRgb(0xbb, 0x06, 0x06);
QColor primaryLightGrayColor = QColor::fromRgb(211, 215, 207);

SubWindow::SubWindow(ros::NodeHandle &nh,
                     QWidget *parent,
                     const std::string &prefix) : QWidget(parent),
                                                  ui(new Ui::SubWindow),
                                                  virtualRM(&virtualRobot, nh, prefix),
                                                  realRM(&realRobot, nh, prefix), m_prefix(prefix) {

    ui->setupUi(this);
    rm = &virtualRM;

    setWindowTitle("JAKA GUI");

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 5; ++j) {
            auto *p = new QLabel("---.-");
            if (i % 2 == 0) {
                p->setAutoFillBackground(true);
                auto palette = p->palette();
                palette.setColor(p->backgroundRole(), primaryLightGrayColor);
                p->setPalette(palette);
            }
            p->setMinimumWidth(42);
            p->setMaximumWidth(42);
            p->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            jointInfoLabelList[i].emplace_back(p);
            ui->jValGridLayout->addWidget(p, i + 1, j + 1);
        }
    }

    signalMapper = new QSignalMapper(this);
    motionControlLabel = new QLabel("运动控制(度)");
    ui->jControlLayout->addWidget(motionControlLabel,
                                  0, 0, 1, 3, Qt::AlignVCenter | Qt::AlignHCenter);
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 2; ++j) {
            auto *p = new QPushButton(j == 0 ? "+" : "-");
            p->setEnabled(false);
            ui->jControlLayout->addWidget(p, i + 1, j);
            p->setMinimumWidth(48);
            p->setMaximumWidth(48);
            connect(p, &QPushButton::clicked,
                    signalMapper, static_cast<void (QSignalMapper::*)(void)>(&QSignalMapper::map));
            signalMapper->setMapping(p, 6 * j + i);
            jointMoveBtList[i].emplace_back(p);
        }
    }
    connect(signalMapper, static_cast<void (QSignalMapper::*)(int)>(&QSignalMapper::mapped),
            this, &SubWindow::onJointMoveBtClicked);

    for (int i = 0; i < 6; ++i) {
        auto *p = new QDoubleSpinBox();
        ui->jControlLayout->addWidget(p, i + 1, 2);
        p->setMinimumWidth(72);
        p->setMaximumWidth(72);
        p->setDecimals(1);
        p->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        // TODO: set joint limit
        p->setMaximum(180);
        p->setMinimum(-180);
//        p->setMinimumHeight(24);
//        p->setMinimumWidth(36);
        jointMoveSpinList.emplace_back(p);
    }

    ui->jControlLayout->setVerticalSpacing(4);
    ui->jControlLayout->setHorizontalSpacing(4);
    ui->jControlLayout->setContentsMargins(4, 0, 0, 0);

    for (int i = 0; i < 6; ++i) {
        auto *p = new QLabel("---.-");
        ui->posGridLayout->addWidget(p, i + 1, 1);
        p->setMinimumWidth(48);
        p->setMaximumWidth(48);
        p->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        posInfoLabelList.emplace_back(p);
    }

    ipEdit = new IPEdit(this);
    ui->ipLayout->addWidget(ipEdit, 3);
    ipEdit->setText("192.168.1.103");

    ui->estopLabel->setAutoFillBackground(true);
    ui->inCollisionLabel->setAutoFillBackground(true);
    ui->powerOnLabel->setAutoFillBackground(true);
    ui->limitLabel->setAutoFillBackground(true);
    ui->dragLabel->setAutoFillBackground(true);
    ui->enableLabel->setAutoFillBackground(true);
    ui->sdkLabel->setAutoFillBackground(true);
    ui->inPosLabel->setAutoFillBackground(true);
    ui->collisionProtectedLabel->setAutoFillBackground(true);
    clearAllStatus();

    auto palette = ui->loginBt->palette();
    palette.setColor(ui->loginBt->backgroundRole(), primaryGreenColor);
    palette.setColor(ui->loginBt->foregroundRole(), QColor::fromRgb(0xff, 0xff, 0xff));
    ui->loginBt->setPalette(palette);

    std::string ip;
    if (!ros::param::get("~" + prefix + "ip", ip)) ip = "192.168.1.100";

    ipEdit->setText(QString::fromStdString(ip));
    ui->typeLabel->setText(QString::fromStdString(prefix));
    rm->set_spin_speed(ui->velocitySpinBox->value() / 180.0 * M_PI);
    ui->robotCheckBox->setChecked(true);

    updateRMConnection(nullptr, rm);

    m_trajectorySrv = nh.advertiseService<qjaka_gui::JointMoveService::Request, qjaka_gui::JointMoveService::Response>(
            m_prefix + "trajectory_srv",
            [&](qjaka_gui::JointMoveService::Request &req, qjaka_gui::JointMoveService::Response &resp) -> bool {
                // init
                resp.success = true;
                // print info
                std::cout << "recv " << m_prefix << "trajectory" << std::endl;
                bprinter::TablePrinter tp(&std::cout);
                tp.AddColumn("", 4);
                for (const auto &joint_name: req.trajectory.joint_trajectory.joint_names) {
                    tp.AddColumn(joint_name, 13);
                }
                tp.PrintHeader();
                for (int i = 0; i < req.trajectory.joint_trajectory.points.size(); ++i) {
                    auto &p = req.trajectory.joint_trajectory.points[i];
                    tp << i;
                    for (double position : p.positions)
                        tp << int(position / M_PI * 180.0 * 10) / 10.0;
                }
                tp.PrintFooter();
                // exec
                rm->set_ee_close(req.ee_close_at_start);
                auto res = rm->trajectory_move(req.trajectory);
                if (res != ERR_SUCC) {
                    resp.success = false;
                    resp.desc.data = DescFactory::getErrorDesc(res);
                } else {
//                    rm->set_ee_close(req.ee_close_at_end);
                }
                // finish
                std::cout << "exec trajectory finished" << std::endl;
                return true;
            });
}

SubWindow::~SubWindow() {
    for (auto &row : jointInfoLabelList)
        for (auto p : row) delete p;
    for (auto &row : jointMoveBtList)
        for (auto p : row) delete p;
    for (auto &p : jointMoveSpinList)
        delete p;
    for (auto p : posInfoLabelList)
        delete p;
    delete ipEdit;
    delete ui;
    delete signalMapper;
    delete motionControlLabel;
}


void SubWindow::onLogin(int errorCode, bool estoped, bool poweredOn, bool servoEnabled) {
    if (errorCode == ERR_SUCC) {
        ui->loginBt->setEnabled(true);
        ui->loginBt->setText("log out");
        if (estoped) {
            showErrorBox(ERR_EMERGENCY_PRESSED);
        }
        onUpdateBt(ERR_SUCC, poweredOn, servoEnabled);
        auto palette = ui->loginBt->palette();
        palette.setColor(ui->loginBt->backgroundRole(), primaryRedColor);
        ui->loginBt->setPalette(palette);
        setLabelStatus(ui->estopLabel, estoped ? STATUS_RED : STATUS_GREEN);
    } else {
        showErrorBox(errorCode);
    }
}

void SubWindow::onLogout(int errorCode) {
    if (errorCode == ERR_SUCC) {
        ui->loginBt->setEnabled(true);
        ui->loginBt->setText("log in");
        ui->powerOnBt->setEnabled(false);
        ui->powerOnBt->setText("power on");
        ui->enableBt->setEnabled(false);
        ui->enableBt->setText("enable");

        auto palette = ui->loginBt->palette();
        palette.setColor(ui->loginBt->backgroundRole(), primaryGreenColor);
        ui->loginBt->setPalette(palette);
        clearAllStatus();
    } else {
        showErrorBox(errorCode);
    }
}


void SubWindow::showErrorBox(int errorCode) {
    if (errorCode != ERR_SUCC)
        QMessageBox::critical(this, "Error",
                              QString::fromStdString(DescFactory::getErrorDesc(errorCode)),
                              QMessageBox::Ok);
}

void SubWindow::setLabelStatus(QLabel *label, emStatusType on) {
    auto palette = label->palette();
    palette.setColor(label->foregroundRole(), QColor(Qt::black));

    switch (on) {
        case STATUS_RED:
            palette.setColor(label->backgroundRole(), primaryRedColor);
            palette.setColor(label->foregroundRole(), QColor(Qt::white));
            break;
        case STATUS_GRAY:
//            palette.setColor(label->foregroundRole(), QColor(Qt::gray));
//            palette.setColor(label->backgroundRole(), QColor::fromRgb(220, 220, 220));
//            break;
        case STATUS_UNKNOWN:
            palette.setColor(label->foregroundRole(), QColor(Qt::gray));
            palette.setColor(label->backgroundRole(), QColor::fromRgb(220, 220, 220));
            break;
        case STATUS_ORANGE:
            palette.setColor(label->backgroundRole(), QColor::fromRgb(255, 140, 0));
            palette.setColor(label->foregroundRole(), QColor(Qt::white));
            break;
        case STATUS_GREEN:
            palette.setColor(label->backgroundRole(), primaryGreenColor);
            palette.setColor(label->foregroundRole(), QColor(Qt::white));

        default:
            break;
    }
    label->setPalette(palette);
}

void SubWindow::on_loginBt_clicked() {
    ui->loginBt->setEnabled(false);
    ui->powerOnBt->setEnabled(false);
    ui->enableBt->setEnabled(false);
    if (!rm->is_login()) {
        ui->loginBt->setText("log in ...");
        rm->login_in(ipEdit->text().toStdString());
    } else {
        ui->loginBt->setText("log out ...");
        rm->login_out();
    }
}

void SubWindow::on_powerOnBt_clicked() {
    qDebug() << "on on_powerOnBt_clicked";
    RobotState s;
    rm->get_robot_state(&s);
    ui->powerOnBt->setEnabled(false);
    ui->enableBt->setEnabled(false);
    if (s.estoped != 1) {
        if (s.poweredOn) {
            ui->powerOnBt->setText("power off...");
            rm->power_off();
        } else {
            qDebug() << "on on_powerOnBt_clicked";
            ui->powerOnBt->setText("power on...");
            rm->power_on();
        }
    }
}


void SubWindow::on_enableBt_clicked() {
    RobotState s;
    rm->get_robot_state(&s);
    ui->powerOnBt->setEnabled(false);
    ui->enableBt->setEnabled(false);
    if (s.estoped != 1) {
        if (s.servoEnabled) {
            ui->enableBt->setText("disable...");
            rm->disable_robot();
        } else {
            ui->enableBt->setText("enable...");
            rm->enable_robot();
        }
    }
}


void SubWindow::clearAllStatus() {
    setLabelStatus(ui->estopLabel, STATUS_UNKNOWN);
    setLabelStatus(ui->inCollisionLabel, STATUS_UNKNOWN);
    setLabelStatus(ui->powerOnLabel, STATUS_UNKNOWN);
    setLabelStatus(ui->limitLabel, STATUS_UNKNOWN);
    setLabelStatus(ui->dragLabel, STATUS_UNKNOWN);
    setLabelStatus(ui->enableLabel, STATUS_UNKNOWN);
    setLabelStatus(ui->sdkLabel, STATUS_UNKNOWN);
    setLabelStatus(ui->inPosLabel, STATUS_UNKNOWN);
    setLabelStatus(ui->collisionProtectedLabel, STATUS_UNKNOWN);
}

/**
 * 更新UI上机器人各状态值
 * @param isAll 是否全量更新
 */
void SubWindow::onUpdateStatus(bool isAll) {
    RobotStatus status;
    tf2::Quaternion qua;
    rm->get_robot_status(&status, qua);
    if (isAll) {
        for (int i = 0; i < 6; ++i) {
            jointInfoLabelList[i][4]->setText(QString::number(status.joint_position[i] / M_PI * 180.0, 'f', 1));
            jointInfoLabelList[i][1]->setText(
                    QString::number(status.robot_monitor_data.jointMonitorData[i].instVoltage, 'f', 1));
            jointInfoLabelList[i][2]->setText(
                    QString::number(status.robot_monitor_data.jointMonitorData[i].instCurrent, 'f', 1));
            jointInfoLabelList[i][3]->setText(
                    QString::number(status.robot_monitor_data.jointMonitorData[i].instTemperature, 'f', 1));
            jointInfoLabelList[i][0]->setText(
                    QString::number(status.torq_sensor_monitor_data.actTorque[i] * 1000.0, 'f', 1));
            double temp = status.cartesiantran_position[i];
            if (i >= 3) temp = temp / M_PI * 180.0;
            posInfoLabelList[i]->setText(QString::number(temp, 'f', 1));
        }

        ui->errorCodeLabel->setText(QString::number(status.errcode));
        ui->tempLabel->setText(QString::number(status.robot_monitor_data.cabTemperature, 'f', 1));
        ui->avgPLabel->setText(QString::number(status.robot_monitor_data.robotAveragePower, 'f', 1));
        ui->avgALabel->setText(QString::number(status.robot_monitor_data.robotAverageCurrent, 'f', 1));
        ui->rapidLabel->setText(QString::number(status.rapidrate, 'f', 2));
        ui->curToolLabel->setText(QString::number(status.current_tool_id));
        auto level = rm->get_collision_level();
        level = (level > 0 && level < 6) ? level : 6;
        ui->collisionLevelComboBox->setCurrentIndex(level);

        setLabelStatus(ui->powerOnLabel, status.powered_on == 0 ? STATUS_RED : STATUS_GREEN);
        setLabelStatus(ui->enableLabel, status.enabled == 0 ? STATUS_RED : STATUS_GREEN);
        setLabelStatus(ui->inPosLabel, status.inpos == 0 ? STATUS_RED : STATUS_GREEN);

        setLabelStatus(ui->estopLabel, status.emergency_stop == 0 ? STATUS_GREEN : STATUS_RED);
        setLabelStatus(ui->inCollisionLabel, status.protective_stop == 0 ? STATUS_GREEN : STATUS_RED);
        setLabelStatus(ui->collisionProtectedLabel, status.protective_stop == 0 ? STATUS_GREEN : STATUS_RED);
        setLabelStatus(ui->limitLabel, status.on_soft_limit == 0 ? STATUS_GREEN : STATUS_RED);
        setLabelStatus(ui->dragLabel, status.drag_status == 0 ? STATUS_GRAY : STATUS_ORANGE);
        setLabelStatus(ui->sdkLabel, status.is_socket_connect == 0 ? STATUS_RED : STATUS_GREEN);

    } else {
        for (int i = 0; i < 6; ++i) {
            jointInfoLabelList[i][4]->setText(QString::number(status.joint_position[i] / M_PI * 180.0, 'f', 1));
        }
    }

    ui->quaXLabel->setText(QString::number(qua.x(), 'f', 4));
    ui->quaYLabel->setText(QString::number(qua.y(), 'f', 4));
    ui->quaZLabel->setText(QString::number(qua.z(), 'f', 4));
    ui->quaWLabel->setText(QString::number(qua.w(), 'f', 4));

    // 字符画
    char text[] = "====";
    if (jointUpdateCount >= strlen(text)) {
        jointUpdateCount = 0;
    } else {
        text[jointUpdateCount++] = '>';
    }
    ui->jointUpdateLabel->setText(text);

    // 用于 set_current
    memcpy(curJVal, status.joint_position, sizeof(curJVal));
}

void SubWindow::onUpdateBt(int errorCode, bool poweredOn, bool servoEnabled) {
    showErrorBox(errorCode);
    ui->powerOnBt->setText(poweredOn ? "power off" : "power on");
    ui->enableBt->setText(servoEnabled ? "disable" : "enable");

    ui->powerOnBt->setEnabled(!servoEnabled);
    ui->enableBt->setEnabled(poweredOn);

    if (servoEnabled) {
        ui->abortBt->setEnabled(true);
        ui->collisionRecoverBt->setEnabled(true);
        ui->setCurrentBt->setEnabled(true);
        ui->goBt->setEnabled(true);
        for (auto &row:jointMoveBtList) {
            for (auto p: row) {
                p->setEnabled(true);
            }
        }
        ui->collisionLevelComboBox->setEnabled(true);
        ui->collisionRecoverBt->setEnabled(true);
    } else {
        ui->abortBt->setEnabled(false);
        ui->collisionRecoverBt->setEnabled(false);
        ui->setCurrentBt->setEnabled(false);
        ui->goBt->setEnabled(false);
        for (auto &row:jointMoveBtList) {
            for (auto p: row) {
                p->setEnabled(false);
            }
        }
        ui->collisionLevelComboBox->setEnabled(false);
        ui->collisionRecoverBt->setEnabled(false);
    }
}

void SubWindow::on_abortBt_clicked() {
    rm->motion_abort();
}


void SubWindow::on_collisionRecoverBt_clicked() {
    rm->collision_recover();
}

void SubWindow::onBusy() {
    QMessageBox::warning(this, "Warning", "上一任务尚未结束", QMessageBox::Ok);
}

void SubWindow::onJointMoveBtClicked(int index) {
    JointValue jVal{};
    jVal.jVal[index % 6] =
            index < 6 ? ui->stepSpinBox->value() / 180.0 * M_PI : -ui->stepSpinBox->value() / 180.0 * M_PI;
    rm->set_spin_speed(ui->velocitySpinBox->value() / 180.0 * M_PI);
    rm->joint_move(&jVal, INCR);
}


void SubWindow::on_setCurrentBt_clicked() {
    for (int i = 0; i < 6; ++i) {
        jointMoveSpinList[i]->setValue(curJVal[i] / M_PI * 180.0);
    }
}


void SubWindow::on_goBt_clicked() {
    JointValue jVal{};
    rm->set_spin_speed(ui->velocitySpinBox->value() / 180.0 * M_PI);
    for (int i = 0; i < 6; ++i) {
        jVal.jVal[i] = jointMoveSpinList[i]->value() / 180.0 * M_PI;
    }
    rm->joint_move(&jVal, ABS);
}

void SubWindow::on_clearInfoBt_clicked() {
    ui->infoList->clear();
}

void SubWindow::on_rapidSpinBox_editingFinished() {
    ui->rapidSlider->setValue(int(ui->rapidSpinBox->value() * 100.0));
}

void SubWindow::on_rapidUpdateBt_clicked() {
    rm->set_rapid(ui->rapidSpinBox->value());
}

void SubWindow::on_rapidSlider_valueChanged(int val) {
    ui->rapidSpinBox->setValue(val / 100.0);
}


void SubWindow::onAddInfo(QString ctx) {
    ui->infoList->addItem(QDateTime::currentDateTime().toString("hh:mm:ss.z") + ": " + ctx);
}

void SubWindow::on_robotCheckBox_toggled(bool useFake) {
    qDebug() << useFake;
    onAddInfo(QString("change to ") + (useFake ? "FAKE" : "REAL") + " robot....");
    RobotManager *old = rm;
    if (useFake) rm = &virtualRM;
    else rm = &realRM;
    updateRMConnection(old, rm);
    if (rm->is_login()) {
        rm->login_in(ipEdit->text().toStdString());
    } else {
        onLogout(ERR_SUCC);
    }
    onAddInfo("change robot complete");
}

void SubWindow::updateRMConnection(RobotManager *old, RobotManager *cur) {
    if (old != nullptr) {
        disconnect(old, &RobotManager::loginSignal, this, &SubWindow::onLogin);
        disconnect(old, &RobotManager::logoutSignal, this, &SubWindow::onLogout);
        disconnect(old, &RobotManager::updateStatusSignal, this, &SubWindow::onUpdateStatus);
        disconnect(old, &RobotManager::errorSignal, this, &SubWindow::showErrorBox);
        disconnect(old, &RobotManager::updateBtSignal, this, &SubWindow::onUpdateBt);
        disconnect(old, &RobotManager::busySignal, this, &SubWindow::onBusy);
        disconnect(old, &RobotManager::addInfoSignal, this, &SubWindow::onAddInfo);
    }
    connect(cur, &RobotManager::loginSignal, this, &SubWindow::onLogin);
    connect(cur, &RobotManager::logoutSignal, this, &SubWindow::onLogout);
    connect(cur, &RobotManager::updateStatusSignal, this, &SubWindow::onUpdateStatus);
    connect(cur, &RobotManager::errorSignal, this, &SubWindow::showErrorBox);
    connect(cur, &RobotManager::updateBtSignal, this, &SubWindow::onUpdateBt);
    connect(cur, &RobotManager::busySignal, this, &SubWindow::onBusy);
    connect(cur, &RobotManager::addInfoSignal, this, &SubWindow::onAddInfo);
}

void SubWindow::on_collisionLevelComboBox_currentIndexChanged(int index) {
    if (index > 0 && index < 6) rm->set_collision_level(index);
    else showErrorBox(ERR_INVALID_PARAMETER);
}


