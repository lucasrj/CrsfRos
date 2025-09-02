
find_path(ASIO_INCLUDE_DIRS
  NAMES asio.hpp
  PATHS /usr/include /usr/local/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ASIO DEFAULT_MSG
  ASIO_INCLUDE_DIRS
)
mark_as_advanced(ASIO_INCLUDE_DIRS)
