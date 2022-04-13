# include dirs for where to get headers from
include_directories("include")
include_directories("./")
# include_directories("../common/include/")
include_directories("../third-party/adafruit-sensors/")
include_directories("../third-party/cxxopts/") # header only lib
include_directories("../third-party/iir/")
include_directories("../third-party/mjbots/moteus/")
include_directories("../third-party/mjbots/pi3hat/")
include_directories("../third-party/nlohmann/") # header only lib
include_directories("../third-party/rapidcsv/") # header only lib
include_directories("../third-party/rapidxml/") # header only lib

include_directories("../lcm-types/cpp")
include_directories("/usr/include/lcm/")   # lcm includes
# include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/include/eigen3")   # lcm includes
include_directories("/opt/vc/include/") # bcm_host
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

link_directories(("/opt/vc/lib/"))

# add_subdirectory(../third-party/mjbots/pi3hat)

# cpp source code to use in building this executable.
# should not include things that get built into library objects elsewhere.
file(GLOB_RECURSE sources
"./*.cpp"
"src/*.cpp")

# find_package(bcm2835)

add_executable(robot_server ${sources} robot_server.cpp robot_server_main.cpp)

target_link_libraries(robot_server pthread lcm I2CDevice Adafruit_ADS1X15 Adafruit_INA260 iir pi3hat -lbcm2835 -lbcm_host)