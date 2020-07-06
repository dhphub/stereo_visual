add_library( svo SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    visual_odometry.cpp
    g2o_types.cpp
    orbextractor.cpp
)

target_link_libraries( svo
    ${THIRD_PARTY_LIBS}
)
