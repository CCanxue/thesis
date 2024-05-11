cmake_minimum_required(VERSION 3.5)

project(Project VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(REALSENSE2_LIBRARY "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64/realsense2.lib")
set(REALSENSE2_INCLUDE_DIR "C:/Program Files (x86)/Intel RealSense SDK 2.0/include")
set(REALSENSE2_LIBRARY_2 "C:/Program Files (x86)/Intel RealSense SDK 2.0/bin/x64/realsense2.dll")
set(OpenCV_DIR "D:/opencv/opencv/opencv-build/install/include")
set(OpenCV_LIBRARY "D:/opencv/opencv/opencv-build/lib/libopencv_core490.dll.a")
set(OpenCV_LIBRARY2 "D:/opencv/opencv/opencv-build/lib/libopencv_imgproc490.dll.a")
set(OpenCV_LIBRARY3 "D:/opencv/opencv/opencv-build/lib/libopencv_highgui490.dll.a")
set(OpenCV_LIBRARY4 "D:/opencv/opencv/opencv-build/lib/libopencv_imgcodecs490.dll.a")
set(OpenCV_LIBRARY5 "D:/opencv/opencv/opencv-build/lib/libopencv_videoio490.dll.a")
set(OPENBLAS_DIR "E:/VSCode+Keil5+STM32CubeMX/Openblas/include")
set(OPENBLAS_LIBRARY "E:/VSCode+Keil5+STM32CubeMX/Openblas/lib/libopenblas.lib")
set(OPENBLAS_LIBRARY2 "E:/VSCode+Keil5+STM32CubeMX/Openblas/lib/libopenblas.dll.a")

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS SerialPort)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core5Compat)

set(PROJECT_SOURCES
        main.cpp
        widget.cpp
        widget.h
        widget.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(Project
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET Project APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(Project SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(Project
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(Project PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(Project PRIVATE Qt${QT_VERSION_MAJOR}::SerialPort)
target_link_libraries(Project PRIVATE Qt${QT_VERSION_MAJOR}::Core5Compat)
target_link_libraries(Project PRIVATE ${REALSENSE2_LIBRARY})
target_link_libraries(Project PRIVATE ${REALSENSE2_LIBRARY_2})
target_include_directories(Project PRIVATE ${OpenCV_DIR})
target_link_libraries(Project PRIVATE ${OpenCV_LIBRARY})
target_link_libraries(Project PRIVATE ${OpenCV_LIBRARY2})
target_link_libraries(Project PRIVATE ${OpenCV_LIBRARY3})
target_link_libraries(Project PRIVATE ${OpenCV_LIBRARY4})
target_link_libraries(Project PRIVATE ${OpenCV_LIBRARY5})
target_link_libraries(Project PRIVATE ${OPENBLAS_LIBRARY})
target_link_libraries(Project PRIVATE ${OPENBLAS_LIBRARY2})
target_include_directories(Project PRIVATE ${OPENBLAS_DIR})
target_include_directories(Project PRIVATE ${REALSENSE2_INCLUDE_DIR})

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.Project)
endif()
set_target_properties(Project PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS Project
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(Project)
endif()
