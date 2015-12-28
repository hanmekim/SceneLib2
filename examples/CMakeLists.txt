############################################################
# SceneLib2 (https://)
FIND_PACKAGE(SceneLib2 REQUIRED)
INCLUDE_DIRECTORIES(${SceneLib2_INCLUDE_DIRS})
LINK_DIRECTORIES(${SceneLib2_LIBRARY_DIRS})
LINK_LIBRARIES(${SceneLib2_LIBRARIES})

# Pangolin (https://github.com/stevenlovegrove/Pangolin)
FIND_PACKAGE(Pangolin REQUIRED)
INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIRS})
LINK_DIRECTORIES(${Pangolin_LIBRARY_DIRS})
LINK_LIBRARIES(${Pangolin_LIBRARIES})

ADD_EXECUTABLE(MonoSlamSceneLib1 MonoSlamSceneLib1.cpp)
#TARGET_LINK_LIBRARIES(MonoSlamSceneLib1 ${OpenCV_LIBS})
