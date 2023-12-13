if (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
  # in cache already
  set(LIBUSB_FOUND TRUE)
else (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)
  find_path(LIBUSB_1_INCLUDE_DIR
    NAMES
libusb.h
    PATHS
      /usr/include
      /usr/local/include
      /sw/include
PATH_SUFFIXES
libusb-1.0
  )

  find_library(LIBUSB_1_LIBRARY
    NAMES
      usb-1.0 usb
    PATHS
      /usr/lib
      /usr/local/lib
      /sw/lib
      /usr/lib/x86_64-linux-gnu
      /usr/lib/aarch64-linux-gnu/
  )

  set(LIBUSB_1_INCLUDE_DIRS    ${LIBUSB_1_INCLUDE_DIR}   )
  set(LIBUSB_1_LIBRARIES     ${LIBUSB_1_LIBRARY})

  if (LIBUSB_1_INCLUDE_DIRS AND LIBUSB_1_LIBRARIES)
     set(LIBUSB_1_FOUND TRUE)
  endif (LIBUSB_1_INCLUDE_DIRS AND LIBUSB_1_LIBRARIES)

  if (LIBUSB_1_FOUND)
    if (NOT libusb_1_FIND_QUIETLY)
      message(STATUS "Found libusb-1.0:")
message(STATUS " - Includes: ${LIBUSB_1_INCLUDE_DIRS}")
message(STATUS " - Libraries: ${LIBUSB_1_LIBRARIES}")
    endif (NOT libusb_1_FIND_QUIETLY)
  else (LIBUSB_1_FOUND)
    if (libusb_1_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find libusb")
    endif (libusb_1_FIND_REQUIRED)
  endif (LIBUSB_1_FOUND)

  # show the LIBUSB_1_INCLUDE_DIRS and LIBUSB_1_LIBRARIES variables only in the advanced view
  mark_as_advanced(LIBUSB_1_INCLUDE_DIRS LIBUSB_1_LIBRARIES)

endif (LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIRS)