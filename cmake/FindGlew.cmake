find_path(GLEW_INCLUDE_DIRS
        NAMES
        GL/glew.h
        PATHS
        ${GLEW_INCLUDE_DIR_HINTS}
        /usr/include
        /usr/local/include
        /sw/include
        /opt/include
        /opt/local/include)
find_library(GLEW_LIBRARIES
        NAMES
        GLEW
        Glew
        glew
        glew32
        PATHS
        ${GLEW_LIBRARY_DIR_HINTS}
        /usr/lib64
        /usr/lib
        /usr/local/lib64
        /usr/local/lib
        /sw/lib
        /opt/lib
        /opt/local/lib)

if(GLEW_INCLUDE_DIRS AND GLEW_LIBRARIES)
    set(GLEW_FOUND TRUE)
    message(STATUS "Found Glew")
    message(STATUS "  Includes : ${GLEW_INCLUDE_DIRS}")
    message(STATUS "  Libraries : ${GLEW_LIBRARIES}")
else()
    set(GLEW_FOUND FALSE)
    if(GLEW_FIND_REQUIRED)
        message(ERROR "Could not find Glew")
    endif()
endif()

mark_as_advanced(GLEW_FOUND)
