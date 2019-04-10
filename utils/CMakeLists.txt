 # CMake file for sba's demo program
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src )
LINK_LIBRARIES(${PROJECT_NAME} ${UCOSLAM_REQUIRED_LIBRARIES})

ADD_EXECUTABLE(ucoslam_stereo stereo_slam.cpp)
ADD_EXECUTABLE(ucoslam_monocular monocular_slam.cpp   inputreader.cpp inputreader.h)
ADD_EXECUTABLE(ucoslam_mapviewer mapviewer.cpp  )
ADD_EXECUTABLE(ucoslam_createinitialparamfile createinitialparamfile.cpp)
ADD_EXECUTABLE(ucoslam_map_removeunusedkeypoint ucoslam_map_removeunusedkeypoint.cpp  )
ADD_EXECUTABLE(ucoslam_stereocalibrate ucoslam_stereocalibrate.cpp  )
ADD_EXECUTABLE(ucoslam_map_export ucoslam_map_export.cpp  )

ADD_EXECUTABLE(ucoslam_pmvs2 ucoslam_pmvs2.cpp  )


INSTALL(TARGETS ucoslam_monocular ucoslam_mapviewer  ucoslam_stereo ucoslam_stereocalibrate ucoslam_pmvs2 RUNTIME DESTINATION bin)

IF(${OPENNI2LIB_FOUND})
    ADD_EXECUTABLE(ucoslam_rgbd rgbd_slam.cpp cvni2.h )
    TARGET_LINK_LIBRARIES(ucoslam_rgbd ${LIBOPENNI2_LIBRARY}  )
    INSTALL(TARGETS ucoslam_rgbd   RUNTIME DESTINATION bin)
ENDIF()
