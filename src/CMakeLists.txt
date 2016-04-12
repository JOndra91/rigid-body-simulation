add_definitions(-DGLM_FORCE_RADIANS)

file(GLOB rigid_body_srcs
	*.cpp
)

file(GLOB rigid_body_hdrs
	*.hpp
)

find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)
find_package(GLEW REQUIRED)

source_group(src FILES ${rigid_body_srcs} ${rigid_body_hdrs})

include_directories(
	${SDL2_INCLUDE_DIR}
	${qu3e_SOURCE_DIR}/src
)

add_executable(rigid_body ${rigid_body_srcs} ${rigid_body_hdrs})

target_link_libraries(rigid_body
	qu3e
	# imgui
	${OPENGL_LIBRARIES}
	${SDL2_LIBRARY}
	${GLEW_LIBRARY}
)
