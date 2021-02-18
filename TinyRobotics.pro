TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        controller.cpp \
        csv_operate.cpp \
        cubic_polynomial_planning.cpp \
        cubic_spline_planning.cpp \
        linear_with_parabola_transition_planning.cpp \
        link.cpp \
        main.cpp \
        matrix.cpp \
        motion_planning.cpp \
        pd_controller.cpp \
        quintic_polynomial_planning.cpp \
        robot.cpp \
        scara.cpp \
        six_dof_robot.cpp \
        torcal_controller.cpp \
        transform_matrix.cpp

HEADERS += \
    controller.h \
    csv_operate.h \
    cubic_polynomial_planning.h \
    cubic_spline_planning.h \
    linear_with_parabola_transition_planning.h \
    link.h \
    matrix.h \
    motion_planning.h \
    pd_controller.h \
    pub_include.h \
    quintic_polynomial_planning.h \
    robot.h \
    scara.h \
    six_dof_robot.h \
    torcal_controller.h \
    transform_matrix.h
