#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "camel-tools-thread" for configuration "Release"
set_property(TARGET camel-tools-thread APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(camel-tools-thread PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcamel-tools-thread.so"
  IMPORTED_SONAME_RELEASE "libcamel-tools-thread.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS camel-tools-thread )
list(APPEND _IMPORT_CHECK_FILES_FOR_camel-tools-thread "${_IMPORT_PREFIX}/lib/libcamel-tools-thread.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
