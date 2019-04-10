include_directories(.)


FILE(GLOB hdrs_base "*.h" )
FILE(GLOB srcs_base "*.cpp")

FILE(GLOB impl_hdrs "impl/*.h" )
FILE(GLOB impl_srcs "impl/*.cpp" )

#SET(impl_hdrs  impl/linear.h    impl/kdtree.h       impl/kmeansindex.h  impl/kmeansindexcreator.h  impl/distances.h impl/heap.h impl/resultset.h  impl/hash.h)
#SET(impl_srcs  impl/linear.cpp impl/kdtree.cpp     impl/kmeansindex.cpp  impl/kmeansindexcreator.cpp  )

FILE(GLOB hdrs ${hdrs_base}  )
FILE(GLOB srcs  ${srcs_base} )
IF(XFLANN_OPENCV)
add_definitions(-DXFLANN_OPENCV)
ENDIF()
SET (LIBNAME ${EXTRALIBNAME}xflann)

ADD_LIBRARY(${LIBNAME} ${srcs} ${hdrs} ${impl_hdrs} ${impl_srcs})

SET_TARGET_PROPERTIES(${LIBNAME} PROPERTIES 				# create *nix style library versions + symbolic links
        DEFINE_SYMBOL XFLANN_DSO_EXPORTS
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_SOVERSION}
	CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
        OUTPUT_NAME "${LIBNAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
)
IF(XFLANN_OPENCV)
target_link_libraries(${LIBNAME}   ${OpenCV_LIBS})
ENDIF()

INSTALL(TARGETS ${LIBNAME}
	RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
	LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
        ARCHIVE DESTINATION lib COMPONENT main)			# Install the dll.a file in lib directory

IF(XFLANN_DEVINSTALL)

INSTALL(FILES ${hdrs_base}
        DESTINATION include/xflann
        COMPONENT main)
ENDIF()

