add_definitions(-DGLM_FORCE_RADIANS)

file(GLOB gmu_srcs
	*.cpp
)

file(GLOB gmu_hdrs
	*.hpp
)

find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)
find_package(GLEW REQUIRED)

source_group(src FILES ${gmu_srcs} ${gmu_hdrs})

include_directories(
	${SDL2_INCLUDE_DIR}
	${qu3e_SOURCE_DIR}/src
)

add_executable(gmu ${gmu_srcs} ${gmu_hdrs})

target_link_libraries(gmu
	qu3e
	imgui
	${OPENGL_LIBRARIES}
	${SDL2_LIBRARY}
	${GLEW_LIBRARY}
)
