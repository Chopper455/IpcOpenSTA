cmake_minimum_required(VERSION 3.10)
project(ipcsta CXX)

message("proj source dir: ${PROJECT_SOURCE_DIR}")

find_package(Boost REQUIRED)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Boost not found!")
endif(Boost_FOUND)

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "YAS submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/external/OpenSTA/CMakeLists.txt")
	message(FATAL_ERROR "The OpenSTA submodule wasn't downloaded!")
endif()
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/external/stalink/CMakeLists.txt")
	message(FATAL_ERROR "The Stalink submodule wasn't downloaded!")
endif()


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(TCL)
if(NOT DEFINED TCL_FOUND OR NOT ${TCL_FOUND})
    message(FATAL_ERROR "Couldn't find the Tcl!")
endif()

if(NOT DEFINED TCL_INCLUDE_PATH OR
	NOT EXISTS ${TCL_INCLUDE_PATH}/tcl.h)
    message(FATAL_ERROR "Couldn't find the 'tcl.h' header, specify TCL_INCLUDE_PATH!")
endif()

#find_package(ZLIB)
include(FindZLIB)
if(NOT DEFINED ZLIB_FOUND OR NOT ${TCL_FOUND})
    message(FATAL_ERROR "Couldn't find the zlib!")
endif()



add_subdirectory(${PROJECT_SOURCE_DIR}/external/OpenSTA)
add_subdirectory(${PROJECT_SOURCE_DIR}/external/stalink)


file(GLOB_RECURSE SRC_SOURCES
	RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}/src/
	${CMAKE_CURRENT_SOURCE_DIR}/external/OpenSTA/include/sta
	${CMAKE_CURRENT_SOURCE_DIR}/external/OpenSTA/search
	${CMAKE_CURRENT_SOURCE_DIR}/external/stalink/src/
	${CMAKE_CURRENT_SOURCE_DIR}/external/stalink/external/yas/include
	${TCL_INCLUDE_PATH}
)

add_executable(${CMAKE_PROJECT_NAME} ${SRC_SOURCES})
#don't need it if OpenSTA also doesn't have it
target_link_libraries(${CMAKE_PROJECT_NAME} stalink)
target_link_libraries(${CMAKE_PROJECT_NAME} OpenSTA )
if (ZLIB_LIBRARIES)
  target_link_libraries(${CMAKE_PROJECT_NAME} ${ZLIB_LIBRARIES})
endif()
target_link_libraries(${CMAKE_PROJECT_NAME} ${TCL_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
if(UNIX AND NOT APPLE)
    target_link_libraries(${CMAKE_PROJECT_NAME} rt)
endif()









