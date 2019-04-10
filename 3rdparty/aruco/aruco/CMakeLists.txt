SET (LIBNAME ${EXTRALIBNAME}aruco)
include_directories(.)


SET(sources
    cameraparameters.cpp  debug.cpp             dictionary.cpp       ippe.cpp    markerdetector.cpp       markerlabeler.cpp  posetracker.cpp
    cvdrawingutils.cpp    dictionary_based.cpp   marker.cpp  markerdetector_impl.cpp  markermap.cpp
    )
SET(headers
    aruco_cvversioning.h  cameraparameters.h  dictionary_based.h  ippe.h            markerdetector_impl.h  markermap.h    timers.h
    aruco_export.h        cvdrawingutils.h    dictionary.h        levmarq.h         marker.h               picoflann.h
    aruco.h               debug.h               markerdetector.h  markerlabeler.h        posetracker.h
    )
IF(BUILD_FRACTALS)
set(fractal_sources
    fractaldetector.cpp
    fractallabelers/fractalposetracker.cpp
    fractallabelers/fractalmarkerset.cpp
    fractallabelers/fractalmarker.cpp
    fractallabelers/fractallabeler.cpp
    )
set(fractal_headers

    fractaldetector.h
    fractallabelers/fractalposetracker.h
    fractallabelers/fractalmarkerset.h
    fractallabelers/fractalmarker.h
    fractallabelers/fractallabeler.h
    )
ENDIF()

add_library(${LIBNAME} ${sources} ${headers} ${fractal_sources} ${fractal_headers})
 
set_target_properties(${LIBNAME} PROPERTIES          # create *nix style library versions + symbolic links
    DEFINE_SYMBOL ARUCO_DSO_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1                       # allow creating static and shared libs without conflicts
    OUTPUT_NAME "${LIBNAME}${PROJECT_DLLVERSION}"    # avoid conflicts between library and binary target names
)

target_link_libraries(${LIBNAME} PUBLIC opencv_core)
IF(BUILD_SVM)
add_definitions(USE_SVM_LABELER)
target_link_libraries(${LIBNAME} PRIVATE opencv_imgproc opencv_calib3d opencv_features2d opencv_ml)
else()
    target_link_libraries(${LIBNAME} PRIVATE opencv_imgproc opencv_calib3d opencv_features2d )
endif()

INSTALL(TARGETS ${LIBNAME}
    RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
    LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
    ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory

IF(ARUCO_DEVINSTALL)
  install(FILES ${headers} ${fractal_headers} DESTINATION include/aruco)
ENDIF()
 
