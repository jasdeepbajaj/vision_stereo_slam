add_library(vslam SHARED camera.cpp dataset.cpp frame.cpp feature.cpp)
target_link_libraries(vslam ${THIRD_PARTY_LIBS})