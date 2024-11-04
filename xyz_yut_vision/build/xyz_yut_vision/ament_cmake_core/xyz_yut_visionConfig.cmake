# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_xyz_yut_vision_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED xyz_yut_vision_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(xyz_yut_vision_FOUND FALSE)
  elseif(NOT xyz_yut_vision_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(xyz_yut_vision_FOUND FALSE)
  endif()
  return()
endif()
set(_xyz_yut_vision_CONFIG_INCLUDED TRUE)

# output package information
if(NOT xyz_yut_vision_FIND_QUIETLY)
  message(STATUS "Found xyz_yut_vision: 0.0.0 (${xyz_yut_vision_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'xyz_yut_vision' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${xyz_yut_vision_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(xyz_yut_vision_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${xyz_yut_vision_DIR}/${_extra}")
endforeach()
