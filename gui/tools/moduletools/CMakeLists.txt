SET(moduletools_SRC modulesetmainwindow.cpp progresswindow.cpp appmodule.cpp appparams.cpp)
SET(moduletools_HDR modulesetmainwindow.h progresswindow.h appmodule.h appparams.h exports.h)
SET(moduletools_UIS modulesetmainwindow.ui progresswindow.ui)
SET(LIBNAME ${EXTRALIBNAME}moduletools)
QT5_WRAP_UI(moduletools_MOC_UIS_APP       ${moduletools_UIS})
ADD_LIBRARY(${LIBNAME} ${moduletools_SRC} ${moduletools_HDR}  ${moduletools_MOC_UIS_APP} )
qt5_use_modules(${LIBNAME} Widgets)
SET_TARGET_PROPERTIES(${LIBNAME} PROPERTIES 				# create *nix style library versions + symbolic links
        DEFINE_SYMBOL APP_MODULETOOLS_DSO_EXPORTS
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_SOVERSION}
        CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
        OUTPUT_NAME "${LIBNAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
)
TARGET_LINK_LIBRARIES(${LIBNAME} ${EXTRALIBNAME}gparam ${EXTRALIBNAME}gparam_qt)
INSTALL(TARGETS ${LIBNAME}
        RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
        ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory
