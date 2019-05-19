TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    main_run.cpp \
    sfm_data_load_help.cpp \
    rotaion_help.cpp \
    linear_constraint_help.cpp

LIBS += \
    /usr/local/lib/lib*.a\
    /usr/local/lib/libopenMVG*.a\
    /usr/local/lib/lib*.so \
    /usr/lib/x86_64-linux-gnu/*.so\
    -lgomp -lpthread \
    /usr/lib/*.so

INCLUDEPATH += /usr/local/include/openMVG \
               /usr/local/include/openMVG/third_party/eigen

HEADERS += \
    camera_lsy.h \
    rotation_help.h \
    sfm_data_load_help.h \
    linear_constraint_help.h

