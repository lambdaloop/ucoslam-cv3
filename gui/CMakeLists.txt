 

#################################
#  Find and Configure Qt5

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Network REQUIRED)

#################################
# Add modules
#################################

include_directories(. ./tools ../src)
SET(BUILD_QT ON)
SET(EXTRALIBNAME "UcoSLAMGUI_")
add_subdirectory(tools/gparam)
add_subdirectory(tools/moduletools)
add_subdirectory(tools/qtgl)

add_subdirectory(videoplayer)
add_subdirectory(modulecalibration)
add_subdirectory(mappingmodule)

#add_subdirectory(mapviewer)
SET(USED_MODULES  UcoSLAMGUI_modulecalibration  UcoSLAMGUI_mappingmodule)


#################################
# Add main files
#################################1

SET(qtproject_SRCS  main.cpp mainwindow.cpp        )
SET(qtproject_HDRS  mainwindow.h  global_gparams.h )


#################################
# Creation rules

QT5_WRAP_UI(qtproject_UIS_HDRS ${qtproject_UIS} )
qt5_add_resources(qtproject_RCC appresources.qrc)

add_executable(UcoSLAM_GUI  ${qtproject_UIS_HDRS} ${qtproject_SRCS}  ${qtproject_RCC} ${qtproject_HDRS} ${qtproject_UIS_HDRS})
qt5_use_modules(UcoSLAM_GUI  Widgets Network)
TARGET_LINK_LIBRARIES(UcoSLAM_GUI   ${QTTOOLS_REQUIRED_LIBRARIES} ${USED_MODULES} ${EXTRALIBNAME}moduletools ${EXTRALIBNAME}gparam ${EXTRALIBNAME}gparam_qt ${EXTRALIBNAME}qtgl ${UCOSLAM_REQUIRED_LIBRARIES} ucoslam)
INSTALL(TARGETS UcoSLAM_GUI  RUNTIME DESTINATION bin)


CONFIGURE_FILE( "${CMAKE_CURRENT_SOURCE_DIR}/installer_script.iss.in" "${CMAKE_CURRENT_SOURCE_DIR}/installer_script.iss" IMMEDIATE @ONLY)
IF(WIN32)
    INSTALL(FILES "program_icon.ico" DESTINATION bin/ )
    INSTALL(FILES "${PROJECT_SOURCE_DIR}/3rdparty/vocabularies/orb.fbow" DESTINATION bin/ )
elseif(UNIX)
#    INSTALL(FILES "program_icon.ico" DESTINATION share/${PROJECT_NAME} )
    INSTALL(FILES "${PROJECT_SOURCE_DIR}/3rdparty/vocabularies/orb.fbow" DESTINATION share/${PROJECT_NAME} )

ENDIF()
