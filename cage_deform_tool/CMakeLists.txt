file(GLOB _SRCS *.cpp)

add_library(cage_deform_tool ${_SRCS})
target_link_libraries(cage_deform_tool ${OpenCV_LIBS})
