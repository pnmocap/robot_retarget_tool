add_library(librobot STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/robot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ik.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/jointnode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mathutils.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/FscAlgorithmImpl.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/MotionAdaptorLib.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/robot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ik.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/jointnode.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mathutils.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/FscAlgorithmImpl.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/MotionAdaptorLib.cpp
)
target_include_directories(librobot PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(librobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/deps/glm)
target_include_directories(librobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/deps/json/single_include)
target_include_directories(librobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/deps/eigen-3.4.0)
target_compile_options(librobot PRIVATE ${MOCAP_API_COMPILER_FLAGS})
