QT   += core gui


greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    HFD_OPEN.h \
    HFD_API_global.h \
    hfdDefines.h \
    hfdVector.h \
    hfdVector.inl

SOURCES += \
     main.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

CONFIG+=debug_and_release

CONFIG(debug,debug|release){
    DESTDIR=$$PWD/build/Debug
unix:!macx: LIBS += -L$$PWD/build/Debug/ -lHFD_APId

INCLUDEPATH += $$PWD/build/Debug
DEPENDPATH += $$PWD/build/Debug


}else{
    DESTDIR=$$PWD/build/Release
unix:!macx: LIBS+=-L$$PWD/build/Release/ -lHFD_API
    INCLUDEPATH += $$PWD/build/Release/
    DEPENDPATH += $$PWD/build/Release/
}


OBJECTS_DIR=$$DESTDIR/obj
MOC_DIR=$$DESTDIR/moc

FORMS +=




