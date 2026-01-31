#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "plan_env::plan_env" for configuration "Release"
set_property(TARGET plan_env::plan_env APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(plan_env::plan_env PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libplan_env.so"
  IMPORTED_SONAME_RELEASE "libplan_env.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS plan_env::plan_env )
list(APPEND _IMPORT_CHECK_FILES_FOR_plan_env::plan_env "${_IMPORT_PREFIX}/lib/libplan_env.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
