#include "mainwindow.h"

#include "./ui_mainwindow.h"


QColor primaryGreenColor = QColor::fromRgb(0x39, 0xa3, 0x0e);
QColor primaryRedColor = QColor::fromRgb(0xbb, 0x06, 0x06);

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 5; ++j) {
            auto *p = new QLabel();
            ui->jValGridLayout->addWidget(p, i + 1, j + 1);
            p->setText("---.-");
            p->setMinimumWidth(48);
            p->setMaximumWidth(48);
            p->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            jointInfoLabelList[i].emplace_back(p);
        }
    }
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 2; ++j) {
            auto *p = new QPushButton();
            ui->jValGridLayout->addWidget(p, i + 1, j + 6);
            p->setText(j == 0 ? "+" : "-");
            p->setMinimumWidth(48);
            p->setMaximumWidth(48);
            jointMoveBtList[i].emplace_back(p);
        }
    }
    for (int i = 0; i < 6; ++i) {
        auto *p = new QDoubleSpinBox();
        ui->jValGridLayout->addWidget(p, i + 1, 8);
        p->setMinimumWidth(72);
        p->setMaximumWidth(72);
        p->setDecimals(1);
        p->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        // TODO: set joint limit
        p->setMaximum(180);
        p->setMinimum(-180);
        jointMoveSpinList[i].emplace_back(p);
    }

    for (int i = 0; i < 6; ++i) {
        auto *p = new QLabel();
        ui->posGridLayout->addWidget(p, i + 1, 1);
        p->setText("---.-");
        p->setMinimumWidth(48);
        p->setMaximumWidth(48);
        p->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        posInfoLabelList.emplace_back(p);
    }

    ipEdit = new IPEdit(this);
    ui->gridLayout->addWidget(ipEdit, 0, 4);
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


    connect(&rm, &RobotManager::loginSignal, this, &MainWindow::onLogin);
    connect(&rm, &RobotManager::logoutSignal, this, &MainWindow::onLogout);
    connect(&rm, &RobotManager::updateStatusSignal, this, &MainWindow::onUpdateStatus);
    connect(&rm, &RobotManager::errorSignal, this, &MainWindow::showErrorBox);
    connect(&rm, &RobotManager::updateBtSignal, this, &MainWindow::onUpdateBt);
}

MainWindow::~MainWindow() {
    delete ui;
    for (auto &row : jointInfoLabelList)
        for (auto p : row) delete p;
    for (auto &row : jointMoveBtList)
        for (auto p : row) delete p;
    for (auto &row : jointMoveSpinList)
        for (auto p : row) delete p;
    for (auto p : posInfoLabelList)
        delete p;
}


void MainWindow::onLogin(int errorCode, bool estoped, bool poweredOn, bool servoEnabled) {
    if (errorCode == ERR_SUCC) {
        ui->loginBt->setEnabled(true);
        ui->loginBt->setText("log out");
        ui->powerOnBt->setText(poweredOn ? "power off" : "power on");
        ui->powerOnBt->setEnabled(true);
        ui->enableBt->setText(servoEnabled ? "disable" : "enable");
        ui->enableBt->setEnabled(poweredOn);
        auto palette = ui->loginBt->palette();
        palette.setColor(ui->loginBt->backgroundRole(), primaryRedColor);
        ui->loginBt->setPalette(palette);

        rm.start_get_thread();
    } else {
        showErrorBox(errorCode);
    }
}

void MainWindow::onLogout(int errorCode) {
    if (errorCode == ERR_SUCC) {
        rm.stop_get_thread();

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


void MainWindow::showErrorBox(int errorCode) {
    if (errorCode != ERR_SUCC)
        QMessageBox::critical(this, "Error",
                              QString::fromStdString(rm.get_error_desc(errorCode)),
                              QMessageBox::Ok);
}

void MainWindow::setLabelStatus(QLabel *label, emStatusType on) {
    auto palette = label->palette();
    palette.setColor(label->foregroundRole(), QColor(Qt::black));

    switch (on) {
        case STATUS_RED:
            palette.setColor(label->backgroundRole(), QColor(Qt::red));
            break;
        case STATUS_GRAY:
            palette.setColor(label->backgroundRole(), QColor::fromRgb(220, 220, 220));
            break;
        case STATUS_UNKNOWN:
            palette.setColor(label->foregroundRole(), QColor(Qt::gray));
            palette.setColor(label->backgroundRole(), QColor::fromRgb(220, 220, 220));
            break;
        case STATUS_ORANGE:
            palette.setColor(label->backgroundRole(), QColor::fromRgb(255, 140, 0));
            break;
        case STATUS_GREEN:
            palette.setColor(label->backgroundRole(), QColor(Qt::green));
        default:
            break;
    }
    label->setPalette(palette);
}

void MainWindow::on_loginBt_clicked() {
    if (!rm.is_login()) {
        ui->loginBt->setText("log in ...");
        ui->loginBt->setEnabled(false);
        rm.login_in(ipEdit->text().toStdString().c_str());
    } else {
        ui->loginBt->setText("log out ...");
        ui->loginBt->setEnabled(false);
        rm.login_out();
    }
}

void MainWindow::on_powerOnBt_clicked() {
    RobotState s;
    rm.get_robot_state(&s);
    if (s.estoped != 1) {
        ui->powerOnBt->setEnabled(false);
        if (s.poweredOn) {
            ui->powerOnBt->setText("power off...");
            rm.power_off();
        } else {
            ui->powerOnBt->setText("power on...");
            rm.power_on();
        }
    }
}


void MainWindow::on_enableBt_clicked() {
    RobotState s;
    rm.get_robot_state(&s);
    if (s.estoped != 1) {
        ui->enableBt->setEnabled(false);
        if (s.servoEnabled) {
            ui->enableBt->setText("disable...");
            rm.disable_robot();
        } else {
            ui->enableBt->setText("enable...");
            rm.enable_robot();
        }
    }

}


void MainWindow::clearAllStatus() {
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

void MainWindow::onUpdateStatus(bool isAll) {
    RobotStatus status;
    rm.get_all_status(&status);
    if (isAll) {
        for (int i = 0; i < 6; ++i) {
            jointInfoLabelList[i][0]->setText(QString::number(status.joint_position[i], 'f', 1));
            jointInfoLabelList[i][1]->setText(
                    QString::number(status.robot_monitor_data.jointMonitorData[i].instVoltage, 'f', 1));
            jointInfoLabelList[i][2]->setText(
                    QString::number(status.robot_monitor_data.jointMonitorData[i].instCurrent, 'f', 1));
            jointInfoLabelList[i][3]->setText(
                    QString::number(status.robot_monitor_data.jointMonitorData[i].instTemperature, 'f', 1));
            jointInfoLabelList[i][4]->setText(QString::number(status.torq_sensor_monitor_data.actTorque[i], 'f', 1));
            posInfoLabelList[i]->setText(QString::number(status.cartesiantran_position[i], 'f', 1));
        }

        ui->errorCodeLabel->setText(QString::number(status.errcode));
        ui->tempLabel->setText(QString::number(status.robot_monitor_data.cabTemperature, 'f', 1));
        ui->avgPLabel->setText(QString::number(status.robot_monitor_data.robotAveragePower, 'f', 1));
        ui->avgALabel->setText(QString::number(status.robot_monitor_data.robotAverageCurrent, 'f', 1));
        ui->rapidLabel->setText(QString::number(status.rapidrate));

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
            jointInfoLabelList[i][0]->setText(QString::number(status.joint_position[i], 'f', 1));
        }
    }
}

void MainWindow::onUpdateBt(int errorCode, bool poweredOn, bool servoEnabled) {
    showErrorBox(errorCode);
    ui->powerOnBt->setText(poweredOn ? "power off" : "power on");
    ui->powerOnBt->setEnabled(true);
    ui->enableBt->setText(servoEnabled ? "disable" : "enable");
    ui->enableBt->setEnabled(poweredOn);
}

void MainWindow::on_abortBt_clicked() {

}


void MainWindow::on_collisionRecoverBt_clicked() {

}

