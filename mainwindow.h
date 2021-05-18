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

#include <vector>
#include <unordered_map>

#include "IPEdit.h"
#include "robot.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

private slots:

    void onLogin(int errorCode, bool estoped, bool poweredOn, bool servoEnabled);

    void onLogout(int errorCode);

    void onUpdateBt(int errorCode, bool poweredOn, bool servoEnabled);

    void on_loginBt_clicked();

    void on_powerOnBt_clicked();

    void on_enableBt_clicked();


private:
    Ui::MainWindow *ui;

    std::vector<QLabel *> jointInfoLabelList[6];
    std::vector<QPushButton *> jointMoveBtList[6];
    std::vector<QDoubleSpinBox *> jointMoveSpinList[6];
    std::vector<QLabel *> posInfoLabelList;
    IPEdit *ipEdit;
    RobotManager rm;

    enum emStatusType {
        STATUS_GREEN, STATUS_RED, STATUS_ORANGE, STATUS_UNKNOWN, STATUS_GRAY
    };

    void showErrorBox(int errorCode);

    void setLabelStatus(QLabel *label, emStatusType on);

    void clearAllStatus();

    void onUpdateStatus(bool isAll);

};

#endif // MAINWINDOW_H
