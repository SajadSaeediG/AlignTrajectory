cmake_minimum_required(VERSION 2.8.0)

project(AlignTrajectoy)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost 1.54 REQUIRED COMPONENTS program_options regex)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)

include_directories(
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}     -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter -Wno-error=unused-variable -Wno-error=unused-but-set-variable -std=c++11")


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


include_directories(src)

add_executable(align src/main.cpp src/AlignTrajectory.cpp src/Viewer.cpp)
target_link_libraries (align ${OpenCV_LIBS}
			     ${Pangolin_LIBRARIES}
			     ${Boost_LIBRARIES})



