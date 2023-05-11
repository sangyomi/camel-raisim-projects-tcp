#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "camel-tools-filter" for configuration "Release"
set_property(TARGET camel-tools-filter APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(camel-tools-filter PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcamel-tools-filter.so"
  IMPORTED_SONAME_RELEASE "libcamel-tools-filter.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS camel-tools-filter )
list(APPEND _IMPORT_CHECK_FILES_FOR_camel-tools-filter "${_IMPORT_PREFIX}/lib/libcamel-tools-filter.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
