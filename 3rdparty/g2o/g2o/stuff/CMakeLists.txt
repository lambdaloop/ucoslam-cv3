ADD_LIBRARY( ${EXTRALIBNAME}g2o_stuff
color_macros.h  macros.h  misc.h  os_specific.c  os_specific.h  property.cpp  property.h  string_tools.cpp  string_tools.h  timeutil.cpp  timeutil.h)


SET_TARGET_PROPERTIES( ${EXTRALIBNAME}g2o_stuff PROPERTIES 				# create *nix style library versions + symbolic links
        DEFINE_SYMBOL stuff_EXPORTS
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_SOVERSION}
        CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
        OUTPUT_NAME "${THIS_LIBNAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
)
INSTALL(TARGETS ${EXTRALIBNAME}g2o_stuff
    RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
    LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
    ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory

