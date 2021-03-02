cmake_minimum_required(VERSION 3.1)
project(path_continuity
  LANGUAGES CXX
  VERSION 0.2.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-fPIC)

find_package(Eigen3 REQUIRED)
find_package(Bezier REQUIRED)
find_package(dlib REQUIRED)
find_package(Qt5 REQUIRED Core Gui Widgets PrintSupport)

set(QT_LIBRARIES Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport)

include_directories(
    SYSTEM
    src
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
    src/qgraphicsviewzoom.cpp
    src/customscene.cpp
    src/qcurve.cpp
    src/qcustomplot.cpp
    src/wheel.cpp
    MotionModes/crab.cpp
    MotionModes/tangent_aligned.cpp
    MotionModes/motion_mode.cpp
)

set(HEADERS
    src/mainwindow.h
    src/qgraphicsviewzoom.h
    src/customscene.h
    src/qcurve.h
    src/qcustomplot.h
    src/wheel.h
    MotionModes/motion_mode.h
    MotionModes/crab.h
    MotionModes/tangent_aligned.h
)

QT5_WRAP_UI(QT_FORMS_HPP mainwindow.ui)
QT5_WRAP_CPP(QT_MOC_HPP ${HEADERS})

add_executable(path_continuity ${SOURCES} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(path_continuity ${QT_LIBRARIES} bezier dlib::dlib)
