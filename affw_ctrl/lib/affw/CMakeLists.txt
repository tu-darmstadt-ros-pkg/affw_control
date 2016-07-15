cmake_minimum_required(VERSION 2.8.3)
project(affw)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")

include(ExternalProject)

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

ExternalProject_Add(
	LIB_FANN
	PREFIX fann
	GIT_REPOSITORY https://github.com/libfann/fann.git
	UPDATE_COMMAND ""
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/fann/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/fann/lib)

ExternalProject_Add(
	LIB_OTL
	PREFIX otl
	GIT_REPOSITORY https://bitbucket.org/haroldsoh/otl.git
	UPDATE_COMMAND ""
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
	INSTALL_COMMAND bash -c "mkdir -p <INSTALL_DIR>/lib <INSTALL_DIR>/include && cp <BINARY_DIR>/libOTL.a <INSTALL_DIR>/lib && cp <SOURCE_DIR>/src/libOTL/*.h <INSTALL_DIR>/include"
)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/otl/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/otl/lib)

find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(include)

add_library(affw
			src/Config.cpp
			src/otl/STORKGP_async.cpp
			src/otl/OESGPasync.cpp
			src/otl/otl_rls_osn.cpp
			src/learner/LWPRLearner.cpp 
			src/learner/DummyLearner.cpp 
			src/learner/ModelLearner.cpp 
			src/learner/FANNLearner.cpp
			src/learner/WrapperLearner.cpp
			src/learner/FeedbackController.cpp 
			src/learner/OESGPLearner.cpp 
			src/learner/STORKGPLearner.cpp 
			src/learner/SOGPLearner.cpp 
			src/learner/RLSLearner.cpp 
			src/learner/RLSESNLearner.cpp 
			src/mapping/DataMapper.cpp
			src/mapping/KTermStateTarget2ActionCompMapper.cpp
			src/util/DelayEstimator.cpp
			src/util/affw_factory.cpp
			)
target_link_libraries(affw lwpr fann OTL ${Boost_LIBRARIES})
add_dependencies(affw LIB_LWPR LIB_FANN LIB_OTL)

#add_executable(testLearner src/test/testLearner.cpp)
#target_link_libraries(testLearner affw)

#add_executable(batchLearnFann src/test/batchLearnFann.cpp)
#target_link_libraries(batchLearnFann fann)
#add_dependencies(batchLearnFann LIB_FANN)

#add_executable(batchLearnOTL src/test/batchLearnOTL.cpp)
#target_link_libraries(batchLearnOTL fann OTL)
#add_dependencies(batchLearnOTL LIB_FANN LIB_OTL)

#add_executable(testDelayEstimator src/test/testDelayEstimator.cpp)
#target_link_libraries(testDelayEstimator affw)

#install(TARGETS affw
#        RUNTIME DESTINATION bin
#        LIBRARY DESTINATION lib
#        ARCHIVE DESTINATION lib)
        
#install(DIRECTORY include DESTINATION include)
