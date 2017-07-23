INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL 
)

LINK_LIBRARIES(
OpenGLSupport BulletDynamics  BulletCollision LinearMath    ${GLUT_glut_LIBRARY} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

ADD_EXECUTABLE(AppForkLiftDemo
		ForkLiftDemo.cpp
		main.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/build/bullet.rc
)

add_executable(AppTgBoxAnchorDebug
    tgBoxAnchorDebugModel.cpp
    AppTgBoxAnchorDebugDemo.cpp
) 

