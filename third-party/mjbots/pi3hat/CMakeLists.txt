include_directories("/opt/vc/include/") # bcm_host
add_library (pi3hat STATIC pi3hat.h pi3hat.cc)
target_link_libraries(pi3hat -lbcm_host)