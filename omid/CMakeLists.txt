# Set the minimum version of CMake that can be used
# To find the cmake version run
# $ cmake --version
cmake_minimum_required(VERSION 3.5)

# Set the project name
project (omid)

# Create a sources variable with a link to all cpp files to compile
file(GLOB SOURCES
    ai/*.h
    ai/Protobuf/*.h
    ai/*.cpp
    ai/Protobuf/*.cc
)
#ai/graphical/*.h
#ai/graphical/*.cpp
#home/mhz/Desktop/omid/ai/GL/*.h

if(NOT "${_PROTOBUF_PROTOC_EXECUTABLE_VERSION}" VERSION_EQUAL "${Protobuf_VERSION}")
   message(WARNING "Protobuf compiler version ${_PROTOBUF_PROTOC_EXECUTABLE_VERSION}"
          " doesn't match library version ${Protobuf_VERSION}")
endif()

FIND_PACKAGE(Protobuf REQUIRED)
#include(FindProtobuf)
#find_package(Protobuf REQUIRED)
#include_directories(${PROTOBUF_INCLUDE_DIR})

# Add an executable with the above sources
add_executable(omid ${SOURCES})

# Set the directories that should be included in the build command for this target
# when running g++ these will be included as -I/directory/path/
#target_include_directories(omid
#    PRIVATE 
#        ${PROJECT_SOURCE_DIR}/ai/Protobuf
#)
target_include_directories(omid PUBLIC
                           	${PROJECT_BINARY_DIR}
                           	${PROJECT_SOURCE_DIR}/ai/Protobuf
                            	${Boost_FILESYSTEM_LIBRARY}
    				${Boost_SYSTEM_LIBRARY}
    				${PROTOBUF_LIBRARY}
                           )
target_link_libraries(omid ${PROTOBUF_LIBRARIES})
