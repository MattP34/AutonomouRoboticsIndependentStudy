# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_all_points_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED all_points_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(all_points_FOUND FALSE)
  elseif(NOT all_points_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(all_points_FOUND FALSE)
  endif()
  return()
endif()
set(_all_points_CONFIG_INCLUDED TRUE)

# output package information
if(NOT all_points_FIND_QUIETLY)
  message(STATUS "Found all_points: 0.0.0 (${all_points_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'all_points' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${all_points_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(all_points_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${all_points_DIR}/${_extra}")
endforeach()
