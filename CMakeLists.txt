cmake_minimum_required(VERSION 3.10)
project(ipcsta CXX)

####################################
#USE_EXTERNAL_LIBS - flag to use external yas, opensta and stalink
#if USE_EXTERNAL_LIBS isn't set, then picks them from git submodules

####################################

message("proj source dir: ${PROJECT_SOURCE_DIR}")

find_package(Boost REQUIRED)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Boost not found!")
endif(Boost_FOUND)


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


message("TCL_LIBRARY1=${TCL_LIBRARY}")
message("TCL_ROOT=${TCL_ROOT}")

#searching TCL if it's variables weren't preset
if(NOT DEFINED TCL_FOUND)
	find_package(TCL)
	if(NOT DEFINED TCL_FOUND OR NOT ${TCL_FOUND})
	    message(FATAL_ERROR "Couldn't find the Tcl!")
	endif()
else()
	if(NOT DEFINED TCL_ROOT)
	    message(FATAL_ERROR "Use TCL_ROOT to pass search path of TCL library!")
	endif()

	find_library(TCL_LIBRARY tcl86 tcl8.6 tcl85 tcl8.5 HINTS ${TCL_ROOT})
	if(NOT DEFINED TCL_LIBRARY OR ${TCL_LIBRARY} STREQUAL TCL_LIBRARY-NOTFOUND)
	    message(FATAL_ERROR "Couln't locate TCL library file!")
	endif()
endif()


message("TCL_LIBRARY2=${TCL_LIBRARY}")

if(NOT DEFINED TCL_INCLUDE_PATH OR
	NOT EXISTS ${TCL_INCLUDE_PATH}/tcl.h)
    message(FATAL_ERROR "Couldn't find the 'tcl.h' header, specify TCL_INCLUDE_PATH!")
endif()

message(STATUS "Using TCL_LIBRARY=${TCL_LIBRARY}")
message(STATUS "Using TCL_INCLUDE_PATH=${TCL_INCLUDE_PATH}")

#find_package(ZLIB)
include(FindZLIB)
if(NOT DEFINED ZLIB_FOUND OR NOT ${TCL_FOUND})
    message(FATAL_ERROR "Couldn't find the zlib!")
endif()

#setting default lib names/paths to use
set(OPENSTA_LIB OpenSTA)
set(STALINK_LIB stalink-static)

#getting dependencies from git submodules if user hasn't told that he has them
if(NOT DEFINED USE_EXTERNAL_LIBS OR NOT ${USE_EXTERNAL_LIBS})
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
	if(NOT EXISTS "${PROJECT_SOURCE_DIR}/external/Stalink/CMakeLists.txt")
		message(FATAL_ERROR "The Stalink submodule wasn't downloaded!")
	endif()

	add_subdirectory(${PROJECT_SOURCE_DIR}/external/OpenSTA)
	add_subdirectory(${PROJECT_SOURCE_DIR}/external/Stalink)

	set(OPENSTA_INCLUDE_PATH
		${CMAKE_CURRENT_SOURCE_DIR}/external/OpenSTA/include/sta
		${CMAKE_CURRENT_SOURCE_DIR}/external/OpenSTA/search)
	set(STALINK_INCLUDE_PATH
		${CMAKE_CURRENT_SOURCE_DIR}/external/Stalink/src/)
	set(YAS_INCLUDE_PATH
		${CMAKE_CURRENT_SOURCE_DIR}/external/Stalink/external/yas/include)
else()
	#picking all paths from external variables, searching libs
	if(NOT DEFINED OPENSTA_INCLUDE_PATH OR NOT DEFINED OPENSTA_LIB_PATH)
	    message(FATAL_ERROR "OpenSta variable OPENSTA_INCLUDE_PATH or OPENSTA_LIB_PATH wasn't preset!")
	endif()

	if(NOT DEFINED STALINK_INCLUDE_PATH OR NOT DEFINED STALINK_LIB_PATH)
	    message(FATAL_ERROR "Stalink variable STALINK_INCLUDE_PATH or STALINK_LIB_PATH wasn't preset!")
	endif()

	if(NOT DEFINED YAS_INCLUDE_PATH)
	    message(FATAL_ERROR "YAS variable YAS_INCLUDE_PATH wasn't preset!")
	endif()
	
	find_library(OPENSTA_LIB OpenSTA HINTS ${OPENSTA_LIB_PATH})
	if(NOT DEFINED OPENSTA_LIB OR ${OPENSTA_LIB} STREQUAL OPENSTA_LIB-NOTFOUND)
	    message(FATAL_ERROR "Couln't locate OpenSTA library file!")
	endif()

	find_library(STALINK_LIB stalink-static HINTS ${STALINK_LIB_PATH})
	if(NOT DEFINED STALINK_LIB_PATH OR ${STALINK_LIB_PATH} STREQUAL STALINK_LIB_PATH-NOTFOUND)
	    message(FATAL_ERROR "Couln't locate stalink library file!")
	endif()
endif()


message("OPENSTA_INCLUDE_PATH=${OPENSTA_INCLUDE_PATH}")
message("OPENSTA_LIB_PATH=${OPENSTA_LIB_PATH}")
message("OPENSTA_LIB=${OPENSTA_LIB}")
message("STALINK_INCLUDE_PATH=${STALINK_INCLUDE_PATH}")
message("STALINK_LIB_PATH=${STALINK_LIB_PATH}")
message("STALINK_LIB=${STALINK_LIB}")
message("YAS_INCLUDE_PATH=${YAS_INCLUDE_PATH}")



file(GLOB_RECURSE SRC_SOURCES
	RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}/src/
	${OPENSTA_INCLUDE_PATH}
	${STALINK_INCLUDE_PATH}
	${YAS_INCLUDE_PATH}
	${TCL_INCLUDE_PATH}
)

add_executable(ipcsta ${SRC_SOURCES})
#don't need it if OpenSTA also doesn't have it
target_link_libraries(ipcsta ${OPENSTA_LIB})
target_link_libraries(ipcsta ${STALINK_LIB})
if (ZLIB_LIBRARIES)
  target_link_libraries(ipcsta ${ZLIB_LIBRARIES})
endif()
target_link_libraries(ipcsta ${TCL_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${EXTERNAL_ADDLINK_LIBS})
if(UNIX AND NOT APPLE)
    target_link_libraries(ipcsta rt)
endif()

###############
# INSTALL
###############
install(TARGETS ipcsta
	DESTINATION bin)







