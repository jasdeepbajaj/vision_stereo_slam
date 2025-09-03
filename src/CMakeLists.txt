add_library(vslam SHARED camera.cpp dataset.cpp frame.cpp feature.cpp mappoint.cpp map.cpp)
target_link_libraries(vslam ${THIRD_PARTY_LIBS})