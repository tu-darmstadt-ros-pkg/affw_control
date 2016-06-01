cmake_minimum_required(VERSION 2.8.3)
project(affw)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(include)
add_library(affw SHARED
			src/Config.cpp
			src/learner/LWPRLearner.cpp 
			src/learner/DummyLearner.cpp 
			src/learner/ModelLearner.cpp 
			src/learner/FANNLearner.cpp
			src/mapping/DataMapper.cpp
			src/mapping/KTermStateTarget2ActionCompMapper.cpp)
target_link_libraries(affw lwpr fann ${Boost_LIBRARIES})

add_executable(affw_test test/testLearner.cpp)
target_link_libraries(affw_test affw)

add_executable(batchLearnFann test/batchLearnFann.cpp)
target_link_libraries(batchLearnFann fann)


install(TARGETS affw
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
        
install(DIRECTORY include/ DESTINATION include/)