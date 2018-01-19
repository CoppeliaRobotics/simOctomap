set(OCTOMAP_LIBPATH
	/opt/ros/indigo/lib
	/usr/lib
	/usr/local/lib
    ${OCTOMAP_DIR}/lib)

find_library(OCTOMAP_LIBRARY octomap PATH ${OCTOMAP_LIBPATH})
if(OCTOMAP_LIBRARY)
    message(STATUS "Found octomap lib at ${OCTOMAP_LIBRARY}")
else()
    message(STATUS "Cannot find octomap lib")
endif()

find_library(OCTOMAP_OCTOMATH_LIBRARY octomath PATH ${OCTOMAP_LIBPATH})
if(OCTOMAP_OCTOMATH_LIBRARY)
    message(STATUS "Found octomath lib at ${OCTOMAP_OCTOMATH_LIBRARY}")
else()
    message(STATUS "Cannot find octomath lib")
endif()

set(OCTOMAP_LIBRARIES ${OCTOMAP_LIBRARY} ${OCTOMAP_OCTOMATH_LIBRARY})

include(CheckIncludeFiles)

find_path(OCTOMAP_INCLUDE_DIR octomap/octomap.h
	PATH /usr/include /opt/ros/indigo/include
    /usr/local/include ${OCTOMAP_DIR}/octomap/include
)
if(OCTOMAP_INCLUDE_DIR)
    message(STATUS "Found octomap headers at ${OCTOMAP_INCLUDE_DIR}")
else()
    message(STATUS "Cannot find octomap headers")
endif()

if(OCTOMAP_LIBRARY AND OCTOMAP_OCTOMATH_LIBRARY AND OCTOMAP_INCLUDE_DIR)
	set(OCTOMAP_FOUND TRUE)
endif()

if(OCTOMAP_FOUND)
	message(STATUS "Found octomap")
else()
	message(STATUS "Octomap not found")
endif()
