cmake_minimum_required(VERSION 3.5)

project(QJAKA_GUI VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check https://doc.qt.io/qt/deployment-android.html for more information.
# They need to be set before the find_package( ...) calls below.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if (APPLE)
    set(Jaka_DIR "/Users/yzy/Documents/iWork/2012-毕设/Projects/QtProjects/jaka_sdk/")
elseif (UNIX)
    set(Jaka_DIR "/home/msi/Opt/jaka_sdk")
endif()
find_package(Jaka REQUIRED)
include_directories(${Jaka_INCLUDE_DIRS})


set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        IPEdit.h
        IPEdit.cpp
        robot.hpp
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(QJAKA_GUI
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(QJAKA_GUI SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(QJAKA_GUI
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(QJAKA_GUI PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${Jaka_LIBRARIES} Threads::Threads)

set_target_properties(QJAKA_GUI PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(QJAKA_GUI)
endif()
