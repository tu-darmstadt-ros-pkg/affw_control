cmake_minimum_required(VERSION 2.8.3)
project(affw)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")
find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(include)
set(AFFW_SRC 
	src/Config.cpp
	src/learner/DummyLearner.cpp 
	src/learner/ModelLearner.cpp 
	src/learner/FeedbackController.cpp 
	src/mapping/DataMapper.cpp
	src/mapping/KTermStateTarget2ActionCompMapper.cpp
	src/util/DelayEstimator.cpp
	src/util/affw_factory.cpp
)
set(AFFW_LIBS ${Boost_LIBRARIES})
set(AFFW_LWPR true)
set(AFFW_FANN false)
set(AFFW_OTL true)
set(AFFW_OSVR false)

################################################################################
# external libraries
include(ExternalProject)

################################################################################
# LWPR
if(${AFFW_LWPR})
ExternalProject_Add(
	LIB_LWPR
	PREFIX lwpr
	URL ${CMAKE_CURRENT_SOURCE_DIR}/lib/lwpr-1.2.6.tar.gz
	BINARY_DIR lwpr/src/LIB_LWPR
	CONFIGURE_COMMAND <SOURCE_DIR>/configure
		--prefix=<INSTALL_DIR>
	INSTALL_COMMAND make install
)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/lwpr/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/lwpr/lib)
set(AFFW_SRC ${AFFW_SRC}
	src/learner/LWPRLearner.cpp 
	src/learner/WrapperLearner.cpp
)
set(AFFW_LIBS ${AFFW_LIBS} lwpr)
add_definitions( -DAFFW_LWPR )
set(AFFW_DEPENDENCIES ${AFFW_DEPENDENCIES} LIB_LWPR)
endif()

################################################################################
# FANN
if(${AFFW_FANN})
ExternalProject_Add(
	LIB_FANN
	PREFIX fann
	GIT_REPOSITORY https://github.com/libfann/fann.git
	UPDATE_COMMAND ""
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/fann/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/fann/lib)
set(AFFW_SRC ${AFFW_SRC}
	src/learner/FANNLearner.cpp
)
set(AFFW_LIBS ${AFFW_LIBS} fann)
add_definitions( -DAFFW_FANN )
set(AFFW_DEPENDENCIES ${AFFW_DEPENDENCIES} LIB_FANN)
endif()

################################################################################
# OTL (SOGP, RLS, ...)
if(${AFFW_OTL})
ExternalProject_Add(
	LIB_OTL
	PREFIX otl
	GIT_REPOSITORY https://g3force@bitbucket.org/g3force/otl.git
	#UPDATE_COMMAND "make -C <SOURCE_DIR>"
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
	INSTALL_COMMAND bash -c "mkdir -p <INSTALL_DIR>/lib <INSTALL_DIR>/include && cp <BINARY_DIR>/libOTL.a <INSTALL_DIR>/lib && cp <SOURCE_DIR>/src/libOTL/*.h <INSTALL_DIR>/include"
)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/otl/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/otl/lib)
set(AFFW_SRC ${AFFW_SRC}
	src/otl/STORKGP_async.cpp
	src/otl/OESGPasync.cpp
	src/otl/otl_rls_osn.cpp
	src/learner/OESGPLearner.cpp 
	src/learner/STORKGPLearner.cpp 
	src/learner/SOGPLearner.cpp 
	src/learner/RLSLearner.cpp 
	src/learner/RLSESNLearner.cpp 
)
set(AFFW_LIBS ${AFFW_LIBS} OTL)
add_definitions( -DAFFW_OTL )
set(AFFW_DEPENDENCIES ${AFFW_DEPENDENCIES} LIB_OTL)
endif()

################################################################################
# O SVR
if(${AFFW_OSVR})
add_library(osvr
			src/osvr/CrossValidation.cpp 
			src/osvr/File.cpp 
			src/osvr/Forget.cpp 
			src/osvr/Kernel.cpp 
			src/osvr/OnlineSVR.cpp 
			src/osvr/Show.cpp 
			src/osvr/Stabilize.cpp 
			src/osvr/Train.cpp 
			src/osvr/Variations.cpp)
set(AFFW_SRC ${AFFW_SRC}
	src/learner/OSVRLearner.cpp
)
set(AFFW_LIBS ${AFFW_LIBS} osvr)
add_definitions( -DAFFW_OSVR )
endif()

################################################################################
# affw library
add_library(affw ${AFFW_SRC})
target_link_libraries(affw ${AFFW_LIBS} )
add_dependencies(affw ${AFFW_DEPENDENCIES} )

################################################################################
# executables for testing
if(${AFFW_TEST})
	add_executable(testLearner src/test/testLearner.cpp)
	target_link_libraries(testLearner affw)
	
	if(${AFFW_FANN})
		add_executable(batchLearnFann src/test/batchLearnFann.cpp)
		target_link_libraries(batchLearnFann fann)
		add_dependencies(batchLearnFann LIB_FANN)
	endif()
	
	if(${AFFW_OTL} AND ${AFFW_FANN})
		add_executable(batchLearnOTL src/test/batchLearnOTL.cpp)
		target_link_libraries(batchLearnOTL fann OTL)
		add_dependencies(batchLearnOTL LIB_FANN LIB_OTL)
	endif()
	
	add_executable(testDelayEstimator src/test/testDelayEstimator.cpp)
	target_link_libraries(testDelayEstimator affw)
	
	if(${AFFW_LWPR})
		add_executable(lwprPerfTest src/test/lwprPerfTest.cpp)
		target_link_libraries(lwprPerfTest lwpr)
	endif()
endif()
