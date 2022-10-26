QT -= gui
QT += core
QT += serialport

CONFIG += c++11 console
CONFIG += static_runtime
CONFIG += thread
CONFIG -= app_bundle
QMAKE_CXXFLAGS_RELEASE += -O2
QMAKE_CXXFLAGS_RELEASE += /MT
QMAKE_CXXFLAGS_RELEASE -= /MD
QMAKE_CXXFLAGS += /MT
QMAKE_CXXFLAGS -= /MD

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        device.cpp \
        main.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    device.h

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/local/lib/release/ -lzml
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/local/lib/debug/ -lzml
else:unix: LIBS += -L$$PWD/../../../../usr/local/lib/ -lzml

INCLUDEPATH += $$PWD/../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../usr/local/include
