cmake_minimum_required(VERSION 3.1.0)
project(fft_wnd)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

message(${CMAKE_MODULE_PATH})

# Find the QtWidgets library
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5PrintSupport CONFIG REQUIRED)
find_package(QCustomPlot CONFIG REQUIRED)
find_package(LibXTRX REQUIRED)

# Populate a CMake variable with the sources
set(mainwindow_SRCS
    mainwindow.ui
    mainwindow.cpp
    rxthread.cpp
    waterfall.cpp
    main.cpp
)

include_directories(${LIBXTRX_INCLUDE_DIRS})

add_executable(mainwindow ${mainwindow_SRCS})
target_link_libraries(mainwindow Qt5::Widgets Qt5::PrintSupport ${QCustomPlot_LIBRARIES} ${LIBXTRX_LIBRARIES})
