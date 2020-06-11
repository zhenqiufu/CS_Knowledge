# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (testthrustallocation)
set(CMAKE_CXX_STANDARD 17)


# UNIX, WIN32, WINRT, CYGWIN, APPLE are environment 
# variables as flags set by default system
if(UNIX)
    message("This is a ${CMAKE_SYSTEM_NAME} system")
elseif(WIN32)
    message("This is a Windows System")
endif()

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


set(CMAKE_INCLUDE_CURRENT_DIR ON)


# thread库
find_package(Threads MODULE REQUIRED)


# 添加 include 子目录


set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../"
	"/home/skloe/catkin_ws4/src/dp_core/third_party/osqp/include"
	"~/catkin_ws4/src/dp_core/third_party/eigen"
	"/opt/mosek/9.0/tools/platform/linux64x86/h"
	)

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
	"/usr/lib"
	"/home/skloe/catkin_ws4/src/dp_core/third_party/osqp/build/out"
	"/opt/mosek/9.0/tools/platform/linux64x86/bin"
    )

# set(SOURCE_FILES ${SOURCE_FILES} 
#	"${PROJECT_SOURCE_DIR}/../../../common/logging/src/easylogging++.cc" )


# 指定生成目标





set(RARE_LIBRARIES ${RARE_LIBRARIES} 
	"boost_system"
	"boost_filesystem"
	"boost_iostreams"
	"util"
	)

 set(osqp_DIR /home/skloe/catkin_ws4/src/dp_core/third_party/osqp/build)



add_executable (testthrust_osqp testthrust_osqp.cc)
target_include_directories(testthrust_osqp PRIVATE ${HEADER_DIRECTORY})
find_package(osqp REQUIRED)
target_link_libraries(testthrust_osqp PUBLIC osqp::osqp)
target_link_libraries(testthrust_osqp PUBLIC ${RARE_LIBRARIES})

# add_executable (testthrust_TAK testthrust_TAK.cc)
# target_include_directories(testthrust_TAK PRIVATE ${HEADER_DIRECTORY})
# find_package(osqp REQUIRED)
# target_link_libraries(testthrust_TAK PUBLIC osqp::osqp)
# target_link_libraries(testthrust_TAK PUBLIC ${RARE_LIBRARIES})

add_executable (testthrust_TAK testthrust_TAK.cc)
target_include_directories(testthrust_TAK PRIVATE ${HEADER_DIRECTORY})
find_library(MOSEK_LIBRARY mosek64 HINTS ${LIBRARY_DIRECTORY})
target_link_libraries(testthrust_TAK PUBLIC ${MOSEK_LIBRARY})
target_link_libraries(testthrust_TAK PUBLIC ${RARE_LIBRARIES})