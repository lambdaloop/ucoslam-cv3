


# ----------------------------------------------------------------------------
#   Program Optimization and debug (Extracted from OpenCV)
# ----------------------------------------------------------------------------
set(WARNINGS_ARE_ERRORS 		OFF CACHE BOOL "Treat warnings as errors")
set(WHOLE_PROGRAM_OPTIMIZATION 	OFF CACHE BOOL "Flags for whole program optimization.")

set(EXTRA_CXX_FLAGS "-march=native -Wall")
find_package(OpenMP  )
#if (OPENMP_FOUND)
#    add_definitions( -DUSE_OMP)
#ENDIF()

 IF(CMAKE_COMPILER_IS_GNUCXX OR MINGW)
 SET(GENERAL_FLAGS " -Wall -std=c++11 -mmmx -msse -msse2 -msse3 ${OpenMP_CXX_FLAGS}")
 add_definitions( -DUSE_SSE)
 IF(WARNINGS_ARE_ERRORS)
     SET(GENERAL_FLAGS   "${GENERAL_FLAGS}  -Werror ")
 ENDIF()
 IF(USE_AVX)
     SET(GENERAL_FLAGS "${GENERAL_FLAGS}  -mavx ")
 ENDIF()
 add_definitions(-DUSE_AVX)
 SET(CMAKE_CXX_FLAGS_RELEASE         "${GENERAL_FLAGS}  -O3 -g0  -DNDEBUG")
 SET(CMAKE_CXX_FLAGS_DEBUG           "${GENERAL_FLAGS}  -O0 -g3  -DDEBUG -D_DEBUG -DPRINT_DEBUG_MESSAGES")
 SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "${GENERAL_FLAGS}  -O1 -g3  -DNDEBUG -DPRINT_DEBUG_MESSAGES")

 ELSE()  # MSVC

 ADD_DEFINITIONS(-DNOMINMAX)

ENDIF()#END OF COMPILER SPECIFIC OPTIONS

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_EXE_LINKER_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${EXTRA_EXE_LINKER_FLAGS_RELEASE}")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${EXTRA_EXE_LINKER_FLAGS_DEBUG}")

