



### OPENGL
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
MESSAGE(STATUS "OPENGL_LIBRARIES=${OPENGL_LIBRARIES}   ")

SET(qtgl_SRC object.cpp  utils.cpp  glviewerscene.cpp transform.cpp glviewer.cpp viewpoint.cpp  glextensions.cpp)
SET(qtgl_HDR qtgl_exports.h   utils.h transform.h viewpoint.h object.h glextensions.h)
SET(qtgl_HDR_QT  glviewerscene.h  glviewer.h )
qt5_add_resources(qtgl_RCC qtgl.qrc)

#    glo_pointcloud.cpp glo_ipm.cpp glo_pointcloud.h glo_ipm.h
SET(LIBNAME ${EXTRALIBNAME}qtgl )
ADD_LIBRARY(${LIBNAME}  ${qtgl_SRC} ${qtgl_HDR} ${qtgl_HDR_QT} ${qtgl_RCC} )
qt5_use_modules(${LIBNAME} Widgets Gui OpenGL)
TARGET_LINK_LIBRARIES(${LIBNAME}   ${OPENGL_LIBRARIES})
 
SET_TARGET_PROPERTIES(${LIBNAME} PROPERTIES 				# create *nix style library versions + symbolic links
        DEFINE_SYMBOL QTGL_DSO_EXPORTS
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_SOVERSION}
        CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
        OUTPUT_NAME "${LIBNAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
)

INSTALL(TARGETS ${LIBNAME}
        RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
        ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory
