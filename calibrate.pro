TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    calibrate.cpp \
    CameraCalibrator.cpp

HEADERS += \
    CameraCalibrator.h

INCLUDEPATH += \
    /usr/local/include \
    /usr/local/include/opencv \
    /usr/local/include/opencv2 \

LIBS +=-L/usr/local/lib -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_viz -lopencv_core

