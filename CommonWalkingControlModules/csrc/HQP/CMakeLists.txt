#
#  Copyright 2010 CNRS
#
#  Author: Florent Lamiraux
#

SET(LIBRARY_NAME ${PROJECT_NAME})
IF(WIN32)
	ADD_DEFINITIONS("-DNOMINMAX")
ENDIF(WIN32)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
# Boost_SIGNALS_LIBRARY

SET(${PROJECT_NAME}_SRC
    ActiveSet.cpp
    Allocator.cpp
    Algebra.cpp
    Bound.cpp
    BaseY.cpp
    HCOD.cpp
    Givens.cpp
    debug.cpp
    Stage.cpp
    BasicStage.cpp
    Random.cpp
)

SET(${PROJECT_NAME}_HEADERS
     api.hpp
     ActiveSet.hpp
     Algebra.hpp
     Allocator.hpp
     BaseY.hpp
     BasicStage.hpp
     Bound.hpp
     debug.hpp
     DestructiveColPivQR.hpp
     Givens.hpp
     HCOD.hpp
     solvers.hpp
     Stage.hpp
     SubMatrix.hpp
     Random.hpp
)

# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/soth")

FOREACH(header ${${PROJECT_NAME}_HEADERS})
  GET_FILENAME_COMPONENT(headerName ${header} NAME)
  IF(WIN32)
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
		${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
		${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/
	)
  ELSE(WIN32)
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
                  ${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
		  ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/${headerName})
  ENDIF(WIN32)
  INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/src/${header}
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)

ENDFOREACH(header)

# Install procedure for the header files
#----------------------------------------------------

# ----------------------------------------------------
# --- LIBS -------------------------------------------
# ----------------------------------------------------

#INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/include)

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${PROJECT_NAME}_SRC})

IF(NOT WIN32)
  TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${Boost_LIBRARIES})
ENDIF(NOT WIN32)

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION "${PROJECT_VERSION}"
  INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION  ${CMAKE_INSTALL_LIBDIR})


