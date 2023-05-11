#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "camel-tools-trajectory" for configuration "Release"
set_property(TARGET camel-tools-trajectory APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(camel-tools-trajectory PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcamel-tools-trajectory.so"
  IMPORTED_SONAME_RELEASE "libcamel-tools-trajectory.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS camel-tools-trajectory )
list(APPEND _IMPORT_CHECK_FILES_FOR_camel-tools-trajectory "${_IMPORT_PREFIX}/lib/libcamel-tools-trajectory.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
