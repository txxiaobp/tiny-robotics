TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        controller.cpp \
        link.cpp \
        main.cpp \
        matrix.cpp \
        pd_controller.cpp \
        robot.cpp \
        scara.cpp \
        six_dof_robot.cpp \
        torcal_controller.cpp \
        transform_matrix.cpp

HEADERS += \
    controller.h \
    link.h \
    matrix.h \
    pd_controller.h \
    pub_include.h \
    robot.h \
    scara.h \
    six_dof_robot.h \
    torcal_controller.h \
    transform_matrix.h
