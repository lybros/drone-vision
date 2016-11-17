list(APPEND FREEIMAGE_CHECK_INCLUDE_DIRS
        ${FREEIMAGE_INCLUDE_DIR_HINTS}
        /usr/include
        /usr/local/include
        /opt/include
        /opt/local/include
        )

list(APPEND FREEIMAGE_CHECK_LIBRARY_DIRS
        ${FREEIMAGE_LIBRARY_DIR_HINTS}
        /usr/lib
        /usr/local/lib
        /opt/lib
        /opt/local/lib
        )

find_path(FREEIMAGE_INCLUDE_DIRS
        NAMES
        FreeImage.h
        PATHS
        ${FREEIMAGE_CHECK_INCLUDE_DIRS})
find_library(FREEIMAGE_LIBRARIES
        NAMES
        freeimage
        PATHS
        ${FREEIMAGE_CHECK_LIBRARY_DIRS})

if(FREEIMAGE_INCLUDE_DIRS AND FREEIMAGE_LIBRARIES)
    set(FREEIMAGE_FOUND TRUE)
endif()

if(FREEIMAGE_FOUND)
    message(STATUS "Found FreeImage")
    message(STATUS "  Includes : ${FREEIMAGE_INCLUDE_DIRS}")
    message(STATUS "  Libraries : ${FREEIMAGE_LIBRARIES}")
else()
    if(FreeImage_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find FreeImage")
    endif()
endif()
