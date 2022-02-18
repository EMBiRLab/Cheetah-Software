cmake_minimum_required(VERSION 3.5)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)
#execute_process(COMMAND ../scripts/make_types.sh)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#set(CMAKE_VERBOSE_MAKEFILE ON)

option(EMBIR_ROBOT_SERVER_BUILD "use compiler flags for EMBiR Lab development" OFF)
option(MINI_CHEETAH_BUILD "use compiler flags for mini cheetah computer" OFF)
option(PERMISSIVE "use -fpermissive compiler flag" OFF)
set(BUILD_TYPE_RELEASE TRUE)

option(NO_SIM "Do not build simulator" OFF)
# SET(def_LINUX "#define linux")
message(STATUS ${CMAKE_SYSTEM_NAME}) 
set(CMAKE_CXX_STANDARD 14)
if(EMBIR_ROBOT_SERVER_BUILD)
  SET (THIS_COM "../" )
  SET(undef_JCQP_USE_AVX2 "#undef JCQP_USE_AVX2")
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
  set(CMAKE_CXX_FLAGS "-no-pie -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo")
  set(CMAKE_C_FLAGS "-ggdb -std=gnu99 -I.")
  message("**** Robot Server build enabled ****")
  add_subdirectory(robot-server)
  # add_subdirectory(common)
  add_subdirectory(third-party/mjbots/pi3hat)
  add_subdirectory(third-party/iir)
  add_subdirectory(third-party/adafruit-sensors)
  
else(EMBIR_ROBOT_SERVER_BUILD)
  if(MINI_CHEETAH_BUILD)
    SET (THIS_COM "../" )
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
      ${CMAKE_BINARY_DIR}/Configuration.h)
    set(CMAKE_CXX_FLAGS "-no-pie -ggdb -std=c++14 -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wshadow -Wsign-promo -Werror")
    set(CMAKE_C_FLAGS "-ggdb -std=gnu99 -I.")
    message("**** Mini-Cheetah build enabled ****")
  else(MINI_CHEETAH_BUILD)
    SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
      ${CMAKE_BINARY_DIR}/Configuration.h)

    if(CMAKE_SYSTEM_NAME MATCHES Linux)
      message("linux general build")
      set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
      -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
      -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
      -Wshadow -Wsign-promo -Werror")
      # set(CMAKE_CXX_FLAGS "-march=native -ggdb -Wall")
    elseif(APPLE)
      set(CMAKE_CXX_FLAGS "-march=native -ggdb -Wall \
      -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
      -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
      -Wshadow -Wsign-promo")
      include_directories("/usr/include/")   # lcm includes
    endif()

    set(CMAKE_C_FLAGS "-ggdb  -march=native -std=gnu99 -I.")
    message("**** Mini-Cheetah build disabled ****")
  endif(MINI_CHEETAH_BUILD)
  add_subdirectory(robot)
  add_subdirectory(third-party)
  add_subdirectory(common)
  if(NO_SIM)

  else(NO_SIM)
    add_subdirectory(sim)
  endif()

  add_subdirectory(user)
  add_subdirectory(rc_test)
  endif(EMBIR_ROBOT_SERVER_BUILD)
  
  
  #find_package(lcm)
  
  if(PERMISSIVE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -Wunused-variable -Wreorder")
  endif()
  
  