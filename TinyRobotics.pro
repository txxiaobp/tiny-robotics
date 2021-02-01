TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        link.cpp \
        main.cpp \
        matrix.cpp \
        robot.cpp \
        scara.cpp \
        six_fod_robot.cpp \
        transform_matrix.cpp

HEADERS += \
    link.h \
    matrix.h \
    pub_include.h \
    robot.h \
    scara.h \
    six_fod_robot.h \
    transform_matrix.h
