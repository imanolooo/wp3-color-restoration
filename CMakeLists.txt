cmake_minimum_required(VERSION 3.17)
project(color_restoration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_PREFIX_PATH "/opt/Qt/5.15.2/gcc_64/lib/cmake")
#set(CMAKE_PREFIX_PATH "/opt/igl/lib/cmake/igl")
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
#set(LIBIGL_DIR "/home/imanol/workspace/libigl/libigl")

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED)


#option(LIBIGL_USE_STATIC_LIBRARY "Use libIGL as static library" ON)
#find_package(LIBIGL REQUIRED QUIET)
#option(LIBIGL_GLFW "Build target igl::glfw" ON)
include(libigl)

add_executable(color_restoration main.cpp MainWindow.cpp MainWindow.h MainWindow.ui PickedColor.cpp PickedColor.h PointCloud.cpp PointCloud.h Point.cpp Point.h happly.h CubeTetrahedron.cpp CubeTetrahedron.h ColorTransformation.cpp ColorTransformation.h ColorTransformation2D.cpp ColorTransformation2D.h)

target_link_libraries(color_restoration
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        igl::core
        )

target_include_directories(color_restoration PRIVATE ../color/src/color)
