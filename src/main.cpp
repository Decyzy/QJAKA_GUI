#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "jaka_gui");

#if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    QApplication a(argc, argv);

//    QFile styleFile( ":/style.qss" );
//    styleFile.open( QFile::ReadOnly );
//    a.setStyleSheet(styleFile.readAll());

    MainWindow w;
    w.show();
    return a.exec();
}
