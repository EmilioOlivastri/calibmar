find_package(OpenCV 4 REQUIRED)
find_package(Qt5 5.14 COMPONENTS Core Gui Widgets REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(calibmar_app WIN32 main.cpp)
add_executable(undistortion_estimation WIN32 undistortion/undistortion_estimation.cpp)
add_executable(undistortion_visualizer WIN32 undistortion/undistortion_visualizer.cpp)
add_executable(undistorter WIN32 undistortion/undistorter.cpp)
add_subdirectory(ui)

set_target_properties(calibmar_app PROPERTIES OUTPUT_NAME calibmar)
set_target_properties(undistortion_estimation PROPERTIES OUTPUT_NAME undistort_estim)
set_target_properties(undistortion_visualizer PROPERTIES OUTPUT_NAME undistort_viz)
set_target_properties(undistorter PROPERTIES OUTPUT_NAME undistorter)

target_include_directories(calibmar_app PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(undistortion_estimation PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(undistortion_visualizer PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(undistorter PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(calibmar_app PRIVATE    
    calibmar
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    ${OpenCV_LIBS}
)

target_link_libraries(undistortion_estimation PRIVATE
    calibmar
    OpenMP::OpenMP_CXX
    ${OpenCV_LIBS}
)

target_link_libraries(undistortion_visualizer PRIVATE
    calibmar
    ${OpenCV_LIBS}
)

target_link_libraries(undistorter PRIVATE
    calibmar
    ${OpenCV_LIBS}
)

install(TARGETS calibmar_app)
install(TARGETS undistortion_estimation)
install(TARGETS undistortion_visualizer)
install(TARGETS undistorter)
