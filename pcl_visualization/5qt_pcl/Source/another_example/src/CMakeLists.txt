cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (pcl-colorize_cloud)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package (VTK REQUIRED)
IF (VTK_FOUND)
    MESSAGE(STATUS "VTK found.")
    INCLUDE(${VTK_USE_FILE})
ENDIF()

find_package (PCL 1.7.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

set  (project_SOURCES main.cpp pclviewer.cpp)
set  (project_HEADERS pclviewer.h)
set  (project_FORMS pclviewer.ui)


ADD_EXECUTABLE  (colorize_cloud ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (colorize_cloud ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

