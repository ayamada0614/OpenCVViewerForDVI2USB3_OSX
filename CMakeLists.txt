PROJECT(OpenCVViewerForDVI2USB3)

cmake_minimum_required(VERSION 2.8)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

FIND_PACKAGE(OpenCV REQUIRED)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_HOME_DIRECTORY}/build/bin")

include_directories(
	${CMAKE_HOME_DIRECTORY}/includes
 	${OpenCV_INCLUDE_DIRS}
)

# --------------------------------------------------------------------------
# Sources
set(SOURCE_FILES
  openCVViewerForDVI2USB3.cxx
)

LINK_DIRECTORIES(${CMAKE_HOME_DIRECTORY}/libs)

# --------------------------------------------------------------------------
# Build and install the library

ADD_EXECUTABLE(
  openCVViewerForDVI2USB3 ${SOURCE_FILES} 
  )

target_link_libraries(
  openCVViewerForDVI2USB3
  ${OpenCV_LIBRARIES}  
  frmgrab.a
  slava.a
  libexpat.a
  libz.1.2.5.dylib
  libjpeg.a
  libpng.a
  libz.a
  stdc++
  "-framework Foundation"
  "-framework Cocoa" objc
  )

