
set(Jaka_FOUND TRUE)


get_filename_component(PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(Jaka_INCLUDE_DIRS ${PREFIX}/include/)
set(Jaka_LIBRARIES ${PREFIX}/lib/libjakaAPI.so)

