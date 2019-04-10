
FILE(GLOB ucoslam_hdrs_base "*.h" )
FILE(GLOB ucoslam_srcs_base "*.cpp")

SET(ucoslam_fextractors_hdr featureextractors/ORBextractor.h featureextractors/feature2dserializable.h )
SET(ucoslam_fextractors_src featureextractors/ORBextractor.cpp featureextractors/feature2dserializable.cpp)
IF(${OpenCV_VERSION} STRGREATER 3)
    SET(ucoslam_fextractors_hdr ${ucoslam_fextractors_hdr} featureextractors/gridextractor.h )
    SET(ucoslam_fextractors_src ${ucoslam_fextractors_src} featureextractors/gridextractor.cpp )
ENDIF()
FILE(GLOB ucoslam_opt_src "optimization/*.cpp" )
FILE(GLOB ucoslam_opt_hdr "optimization/*.h" )
FILE(GLOB ucoslam_map_hdr "map_types/*.h" )
FILE(GLOB ucoslam_map_src "map_types/*.cpp" )
FILE(GLOB ucoslam_utils_hdr "utils/*.h" )
FILE(GLOB ucoslam_utils_src "utils/*.cpp" )
FILE(GLOB ucoslam_basictypes_hdr "basictypes/*.h" )
FILE(GLOB ucoslam_basictypes_src "basictypes/*.cpp" )




ADD_LIBRARY(${PROJECT_NAME}  ${ucoslam_hdrs_base} ${ucoslam_srcs_base} ${ucoslam_fextractors_src} ${ucoslam_fextractors_hdr}  ${ucoslam_basictypes_src} ${ucoslam_basictypes_hdr}  ${ucoslam_map_hdr} ${ucoslam_map_src} ${ucoslam_utils_hdr} ${ucoslam_utils_src} ${ucoslam_opt_src} ${ucoslam_opt_hdr})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES 				# create *nix style library versions + symbolic links
    DEFINE_SYMBOL UCOSLAM_DSO_EXPORTS
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_SOVERSION}
    CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
    OUTPUT_NAME "${PROJECT_NAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
    )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${UCOSLAM_REQUIRED_LIBRARIES} )

INSTALL(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
    LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
    ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory



INSTALL(FILES ${ucoslam_hdrs_base}
    DESTINATION include/${PROJECT_NAME}
    COMPONENT main)

#INSTALL(FILES ${ucoslam_opt_hdr}   DESTINATION include/${PROJECT_NAME}/optimizers   COMPONENT main)
#INSTALL(FILES ${ucoslam_fextractors_hdr} DESTINATION include/${PROJECT_NAME}/featureextractors COMPONENT main)
#INSTALL(FILES ${ucoslam_utils_hdr}       DESTINATION include/${PROJECT_NAME}/utils COMPONENT main)
INSTALL(FILES ${ucoslam_map_hdr}         DESTINATION include/${PROJECT_NAME}/map_types COMPONENT main)
INSTALL(FILES ${ucoslam_basictypes_hdr}         DESTINATION include/${PROJECT_NAME}/basictypes COMPONENT main)

