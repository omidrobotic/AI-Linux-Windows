# Set the minimum version of CMake that can be used
# To find the cmake version run
# $ cmake --version
cmake_minimum_required(VERSION 3.10)

# Set the project name
project (omid)

# Create a sources variable with a link to all cpp files to compile
file(GLOB SOURCES
    ai/*.h
    ai/*.cpp
    ai/Protobuf/ER-force/erforce/*.h
    ai/Protobuf/ER-force/erforce/*.cc
    ai/Protobuf/ER-force/vision/*.h
    ai/Protobuf/ER-force/vision/*.cc
    ai/Protobuf/Vision/*.h
    ai/Protobuf/Vision/*.cc
    ai/Protobuf/Refree/*.h
    ai/Protobuf/Refree/*.cc
    ai/Protobuf/Grsim/*.h
    ai/Protobuf/Grsim/*.cc
    ai/Protobuf/ER-force/*.h
    ai/Protobuf/ER-force/*.cc
		ai/graphical/*.h
		ai/graphical/*.cpp
		ai/GL/*.h
)



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
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

target_link_libraries(omid ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
