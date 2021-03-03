include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# always set this flag to use root !!!!!
SET( CMAKE_EXE_LINKER_FLAGS  "-Wl,--no-as-needed")

add_executable(simpx simpx.cc)
target_link_libraries(simpx libtelescope)
