# executable
set( EXE_NAME CabacTraining )

# get source files
file( GLOB SRC_FILES "*.cpp" )

# get include files
file( GLOB INC_FILES "*.h" )

# add executable
add_executable( ${EXE_NAME} ${SRC_FILES} ${INC_FILES} )

target_link_libraries( ${EXE_NAME} CommonLib Utilities ${ADDITIONAL_LIBS} )

# include the output directory, where the svnrevision.h file is generated
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories(${CMAKE_SOURCE_DIR}/source/Lib)

if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
  add_custom_command( TARGET ${EXE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
                                                          $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}/CabacTraining>
                                                          $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}/CabacTraining>
                                                          $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}/CabacTraining>
                                                          $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}/CabacTraining>
                                                          $<$<CONFIG:Debug>:${CMAKE_SOURCE_DIR}/bin/CabacTrainingd>
                                                          $<$<CONFIG:Release>:${CMAKE_SOURCE_DIR}/bin/CabacTraining>
                                                          $<$<CONFIG:RelWithDebInfo>:${CMAKE_SOURCE_DIR}/bin/CabacTrainingp>
                                                          $<$<CONFIG:MinSizeRel>:${CMAKE_SOURCE_DIR}/bin/CabacTrainingm> )
endif()

# set the folder where to place the projects
set_target_properties( ${EXE_NAME}  PROPERTIES FOLDER app LINKER_LANGUAGE CXX )

