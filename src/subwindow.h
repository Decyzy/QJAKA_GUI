#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QRandomGenerator>
#include <QTimer>
#include <QMessageBox>
#include <QDebug>
#include <QSignalMapper>
#include <QDateTime>

#include <vector>
#include <unordered_map>

#include "IPEdit.h"
#include "robot_manager.hpp"


QT_BEGIN_NAMESPACE
namespace Ui { class SubWindow; }
QT_END_NAMESPACE

class SubWindow : public QWidget {
Q_OBJECT

public:
    SubWindow(ros::NodeHandle &nh, QWidget *parent = nullptr, const std::string &prefix = "unknown");

    ~SubWindow();

private slots:

    void onLogin(int errorCode, bool estoped, bool poweredOn, bool servoEnabled);

    void onLogout(int errorCode);

    void onUpdateBt(int errorCode, bool poweredOn, bool servoEnabled);

    void onBusy();

    void onJointMoveBtClicked(int);

    void onAddInfo(QString);

    void on_loginBt_clicked();

    void on_powerOnBt_clicked();

    void on_enableBt_clicked();


    void on_abortBt_clicked();

    void on_collisionRecoverBt_clicked();

    void on_setCurrentBt_clicked();

    void on_goBt_clicked();

    void on_clearInfoBt_clicked();

    void on_rapidSpinBox_editingFinished();

    void on_rapidSlider_valueChanged(int);

    void on_rapidUpdateBt_clicked();

    void on_robotCheckBox_toggled(bool);

public:
    RobotManager *rm;

private:
    Ui::SubWindow *ui;

    std::vector<QLabel *> jointInfoLabelList[6];
    std::vector<QPushButton *> jointMoveBtList[6];
    std::vector<QDoubleSpinBox *> jointMoveSpinList;
    std::vector<QLabel *> posInfoLabelList;
    IPEdit *ipEdit;
    ros::NodeHandle &nh;
    VirtualRobot virtualRobot;
    RealRobot realRobot;
    RobotManager virtualRM;
    RobotManager realRM;


    QLabel *motionControlLabel;

    double curJVal[6]{};

    QSignalMapper *signalMapper;

    enum emStatusType {
        STATUS_GREEN, STATUS_RED, STATUS_ORANGE, STATUS_UNKNOWN, STATUS_GRAY
    };

    void showErrorBox(int errorCode);

    void setLabelStatus(QLabel *label, emStatusType on);

    void clearAllStatus();

    void onUpdateStatus(bool isAll);

    void updateRMConnection(RobotManager *old, RobotManager *cur);

    int jointUpdateCount = 0;

};

#endif // MAINWINDOW_H
