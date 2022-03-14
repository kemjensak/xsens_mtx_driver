cmake_minimum_required(VERSION 3.3)
project(xsens_mtx_driver)

find_package(catkin REQUIRED)

add_compile_options(-O -Wall -std=c++14 -fpermissive)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    tf
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    src
)

add_library(cpp_files
    src/cmt1.cpp  
    src/cmt2.cpp  
    src/cmt3.cpp  
    src/cmtmessage.cpp  
    src/cmtpacket.cpp  
    src/cmtscan.cpp  
    src/xsens_std.cpp  
    src/xsens_time.cpp
)

add_executable(mtx src/mtx.cpp)
# add_executable(mtx_cal src/mtx_cal.cpp)

add_dependencies(mtx ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(mtx_cal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_compile_options(mtx PUBLIC -O3 -mtune=native -march=native -DNDEBUG -Wall -Wextra -Wno-attributes -lcurses -ltinfo)
target_link_libraries(mtx
    ${catkin_LIBRARIES}
    ${Curses_LIBRARIES}
    ${pthread_LIBRARIES}
    cpp_files
    ncurses
    curses
    
)






