INCLUDE_DIRECTORIES(.)
SET(LIBNAME ${EXTRALIBNAME}gparam)

SET(hdrs gparam.h encryptor.h gparam_exports.h)
SET(srcs encryptor.cpp gparam.cpp)


ADD_LIBRARY(${LIBNAME} ${srcs} ${hdrs})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ) 

SET_TARGET_PROPERTIES(${LIBNAME} PROPERTIES 				# create *nix style library versions + symbolic links
    DEFINE_SYMBOL  GPARAM_DSO_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
    OUTPUT_NAME "${LIBNAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
    )


INSTALL(TARGETS ${LIBNAME}
    RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
    LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
    ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory

IF(GPARAM_DEVINSTALL)
 INSTALL(FILES ${hdrs}  DESTINATION include/gparam COMPONENT main)
ENDIF()

#####################################
#   Qt module
#####################################


IF(BUILD_QT)
SET(QT_SRCS_APP    paramsetwdgt.cpp paramsetdlg.cpp)
SET(QT_HDRS_APP    paramsetwdgt.h paramsetdlg.h)

IF(BUILD_QT4)
    FIND_PACKAGE(Qt4 REQUIRED)
    SET(GPARAM_REQUIRED_LIBRARIES_QT  ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
    INCLUDE(${QT_USE_FILE})
    QT4_WRAP_CPP(MOC_QT_SRCS_APP     ${QT_HDRS_APP})
    QT4_WRAP_UI(MOC_UIS_APP       ${UIS_APP})
    ADD_LIBRARY(${LIBNAME}_qt ${QT_HDRS_APP} ${QT_SRCS_APP} ${MOC_QT_SRCS_APP} ${MOC_UIS_APP} ${RCC_SRCS} )
    TARGET_LINK_LIBRARIES(${LIBNAME}_qt  ${LIBNAME} ${GPARAM_REQUIRED_LIBRARIES_QT} )
    SET(GPARAM_REQUIRED_LIBRARIES_QT ${GPARAM_REQUIRED_LIBRARIES_QT} PARENT_SCOPE)

ELSE()
    set(CMAKE_AUTOMOC ON)
    find_package(Qt5Widgets REQUIRED)
    ADD_LIBRARY(${LIBNAME}_qt ${QT_HDRS_APP} ${QT_SRCS_APP} )
    qt5_use_modules(${LIBNAME}_qt Widgets)
    TARGET_LINK_LIBRARIES(${LIBNAME}_qt   ${LIBNAME} )
ENDIF()

SET(PROJECT_QT_CREATED_FLAG "YES" PARENT_SCOPE)
SET(LIB_COMPONENTS ${LIB_COMPONENTS} ${LIBNAME}_qt PARENT_SCOPE)


SET_TARGET_PROPERTIES(${LIBNAME}_qt PROPERTIES 				# create *nix style library versions + symbolic links
    DEFINE_SYMBOL GPARAM_DSO_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
    OUTPUT_NAME "${LIBNAME}_qt${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
    )


INSTALL(TARGETS ${LIBNAME}_qt
    RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
    LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
    ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory
IF(GPARAM_DEVINSTALL)
 INSTALL(FILES ${QT_HDRS_APP}  DESTINATION include/gparam COMPONENT main)
ENDIF()


ENDIF()
