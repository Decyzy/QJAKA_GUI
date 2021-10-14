#include "mainwindow.h"

#include <QApplication>
#include <signal.h>


QApplication *g_a;
void sigintHandler(int sig) {
    g_a->quit();
    ros::shutdown();
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "jaka_gui", ros::init_options::NoSigintHandler);

#if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    QApplication a(argc, argv);
    g_a = &a;

//    QFile styleFile( ":/style.qss" );
//    styleFile.open( QFile::ReadOnly );
//    a.setStyleSheet(styleFile.readAll());

    MainWindow w;

    signal(SIGINT, sigintHandler);
    w.show();
    return a.exec();
}
