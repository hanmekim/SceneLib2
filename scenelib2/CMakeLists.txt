############################################################
## Required Dependencies
FIND_PACKAGE(Boost COMPONENTS thread filesystem system REQUIRED QUIET)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)

# Apply required dependencies settings
LIST(APPEND USER_INC "${Boost_INCLUDE_DIRS};${Pangolin_INCLUDE_DIRS};${CMAKE_CURRENT_SOURCE_DIR};${CMAKE_CURRENT_BINARY_DIR};${EIGEN3_INCLUDE_DIR};${OpenCV_INCLUDE_DIRS}")
LIST(APPEND LIB_DIRS "${Boost_LIBRARY_DIRS};${Pangolin_LIBRARY_DIRS};${PROJECT_BINARY_DIR}/src;${OpenCV_LIB_DIR}")
LIST(APPEND LINK_LIBS "${Boost_LIBRARIES};${GLUT_LIBRARIES};${Pangolin_LIBRARIES};${OpenCV_LIBS}")

############################################################
## Add Libraries / Include Directories / Link Directories
INCLUDE_DIRECTORIES(${INTERNAL_INC})
INCLUDE_DIRECTORIES(${USER_INC})
LINK_DIRECTORIES(${LIB_DIRS})
LINK_LIBRARIES(${LINK_LIBS})

############################################################
## SceneLib2 Library Sources
SET(SOURCES monoslam.h monoslam.cpp
            motion_model.h motion_model.cpp
            feature.h feature.cpp feature_init_info.h feature_init_info.cpp
            feature_model.h feature_model.cpp
            part_feature_model.h part_feature_model.cpp
            full_feature_model.h full_feature_model.cpp
            camera.h camera.cpp kalman.h kalman.cpp
            framegrabber/framegrabber.h framegrabber/framegrabber.cpp
            framegrabber/filegrabber.h framegrabber/filegrabber.cpp
            framegrabber/usbcamgrabber.h framegrabber/usbcamgrabber.cpp
            graphic/graphictool.h graphic/graphictool.cpp
            support/math_util.h support/math_util.cpp
            support/eigen_util.h support/eigen_util.cpp
            support/pangolin_util.h support/pangolin_util.cpp
            improc/improc.h improc/improc.cpp
            improc/search_multiple_overlapping_ellipses.h
            improc/search_multiple_overlapping_ellipses.cpp)

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SOURCES})

############################################################
## Export config file for easy inclusion into other projects: FIND_PACKAGE(SceneLib2)
CONFIGURE_FILE  (
                ${CMAKE_CURRENT_SOURCE_DIR}/SceneLib2Config.cmake.in
                ${CMAKE_CURRENT_BINARY_DIR}/SceneLib2Config.cmake
                @ONLY IMMEDIATE
                )

EXPORT(PACKAGE SceneLib2)
