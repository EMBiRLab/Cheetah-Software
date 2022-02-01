# include dirs for where to get headers from
include_directories("include")
include_directories("./")
include_directories("../../third-party/")
include_directories("../../third-party/adafruit-sensors")
include_directories("../../third-party/cxxopts") # header only lib
include_directories("../../third-party/iir")
include_directories("../../third-party/mjbots")
include_directories("../../third-party/mjbots/moteus")
include_directories("../../third-party/mjbots/pi3hat")
include_directories("../../third-party/nlohmann") # header only lib
include_directories("../../third-party/rapidcsv") # header only lib
include_directories("../../third-party/rapidxml") # header only lib

include_directories("../../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

# cpp source code to use in building this executable.
# should not include things that get built into library objects elsewhere.
file(GLOB_RECURSE sources
"./*.cpp"
"src/*.cpp")

add_executable(robot_server ${sources} robot_server.cpp robot_server_main.cpp)

target_link_libraries(robot_server I2CDevice Adafruit_ADS1X15 Adafruit_INA260 iir pi3hat)