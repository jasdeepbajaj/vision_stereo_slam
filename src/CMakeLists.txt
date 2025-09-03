add_library(vslam SHARED camera.cpp dataset.cpp frame.cpp feature.cpp map_point.cpp)
target_link_libraries(vslam ${THIRD_PARTY_LIBS})