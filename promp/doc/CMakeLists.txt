# first we can indicate the documentation build as an option and set it to OFF by default
# https://github.com/matusnovak/doxybook2

find_package(Doxygen)
find_program(doxybook_FOUND NAMES doxybook2)

option(BUILD_DOCUMENTATION "Create the markdown API        
documentation (requires Doxygen and Doxybook2)" OFF)


if (BUILD_DOCUMENTATION)
    if(DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/../doc/prompDoxyConfig.conf)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

        # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_IN}
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Generating API documentation (XML) with Doxygen"
            VERBATIM )

        # call doxybook & agregate everything in the right file (docs/api.md)
        if(doxybook_FOUND)
            message("doxybook2 executable found.")
            add_custom_target ( doc_md ALL
                COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/create_docs.sh ${CMAKE_CURRENT_SOURCE_DIR}
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                COMMENT "Creating API documentation in markdown from Doxygen XML"
                VERBATIM)
        else()
            message("doxybook2 executable not found. cannot create md documentation.")
        endif()
    endif()
endif (BUILD_DOCUMENTATION)