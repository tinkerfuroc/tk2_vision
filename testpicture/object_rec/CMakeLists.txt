cmake_minimum_required(VERSION 2.8.3)
project(ObjectFinder)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIRS}
)

set(LIBRARIES
    ${OPENCV_LIBS}
    ${OPENCV_LIBRARIES}
)

set(SOURCE_FILES SparseRecognition.cc)
add_executable(modelbuild ModelBuild.cpp ${SOURCE_FILES})
add_executable(object_classify ObjectClassify.cpp ${SOURCE_FILES})

target_link_libraries(modelbuild
${OpenCV_LIBS}
)

target_link_libraries(object_classify
${OpenCV_LIBS}
)
