cmake_minimum_required(VERSION 3.10)
project(vslam)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++17 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##################################################
#dependencies
##################################################

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

Find_Package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# csparse
find_package_handle_standard_args(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# fmt
find_package(fmt REQUIRED)

set(G2O_LIBS 
    g2o_core 
    g2o_stuff 
    g2o_types_sba 
    g2o_solver_csparse 
    g2o_csparse_extension)

set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${G2O_LIBS}
    pthread
    ${CSPARSE_LIBRARY}
    fmt::fmt)

##################################################
# include directories
##################################################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(app)


#####message if OpenCV is found
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV")
else()
    message(WARNING "OpenCV not found")
endif()

###message if Eigen3 is found
if(Eigen3_FOUND)
    message(STATUS "Found Eigen3")
else()
    message(WARNING "Eigen3 not found")
endif()

###message if Sophus is found
if(Sophus_FOUND)
    message(STATUS "Found Sophus")
else()
    message(WARNING "Sophus not found")
endif()

###message if CSparse is found
if(CSparse_FOUND)
    message(STATUS "Found CSparse")
else()
    message(WARNING "CSparse not found")
endif()

###message if GFlags is found
if(GFlags_FOUND)
    message(STATUS "Found GFlags")
else()
    message(WARNING "GFlags not found")
endif()

###message if Glog is found
if(Glog_FOUND)
    message(STATUS "Found Glog")
else()
    message(WARNING "Glog not found")
endif()

###message if fmt is found
if(fmt_FOUND)
    message(STATUS "Found fmt")
else()
    message(WARNING "fmt not found")
endif()

###message if g2o is found
if(G2O_FOUND)
    message(STATUS "Found g2o")
else()
    message(WARNING "g2o not found")
endif()