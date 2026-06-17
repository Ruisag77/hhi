# executable DecoderApp & DecoderAnalyserApp

# get source files
file( GLOB SRC_FILES "*.cpp" )

# get include files
file( GLOB INC_FILES "*.h" )

# get additional libs for gcc on Ubuntu systems
if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
  if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    if( USE_ADDRESS_SANITIZER )
      set( ADDITIONAL_LIBS asan )
    endif()
  endif()
endif()

# NATVIS files for Visual Studio
if( MSVC )
  file( GLOB NATVIS_FILES "../../VisualStudio/*.natvis" )
endif()

# example: place header files in different folders
source_group( "Natvis Files" FILES ${NATVIS_FILES} )

macro( define_DecoderXApp_target EXE_NAME LINK_LIBS )
  # add executable
  add_executable( ${EXE_NAME} ${SRC_FILES} ${INC_FILES} ${NATVIS_FILES} )
  include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

  if( DEFINED NX2_ENABLE_TRACING )
    if( NX2_ENABLE_TRACING )
      target_compile_definitions( ${EXE_NAME} PUBLIC ENABLE_TRACING=1 )
    else()
      target_compile_definitions( ${EXE_NAME} PUBLIC ENABLE_TRACING=0 )
    endif()
  endif()

  if( CMAKE_COMPILER_IS_GNUCC AND BUILD_STATIC )
    set( ADDITIONAL_LIBS ${ADDITIONAL_LIBS} -static -static-libgcc -static-libstdc++ )
    target_compile_definitions( ${EXE_NAME} PUBLIC ENABLE_WPP_STATIC_LINK=1 )
  endif()

  target_link_libraries( ${EXE_NAME} ${LINK_LIBS} )
  target_link_libraries( ${EXE_NAME} ${ADDITIONAL_LIBS} )

  # lldb custom data formatters
  #if( XCODE )
  #  add_dependencies( ${EXE_NAME} Install${PROJECT_NAME}LldbFiles )
  #endif()

  if( CMAKE_SYSTEM_NAME STREQUAL "Linux" )
    add_custom_command( TARGET ${EXE_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
                        $<$<CONFIG:Debug>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG}/${EXE_NAME}>
                        $<$<CONFIG:Release>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}/${EXE_NAME}>
                        $<$<CONFIG:RelWithDebInfo>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO}/${EXE_NAME}>
                        $<$<CONFIG:MinSizeRel>:${CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL}/${EXE_NAME}>
                        $<$<CONFIG:Debug>:${CMAKE_SOURCE_DIR}/bin/${EXE_NAME}Staticd>
                        $<$<CONFIG:Release>:${CMAKE_SOURCE_DIR}/bin/${EXE_NAME}Static>
                        $<$<CONFIG:RelWithDebInfo>:${CMAKE_SOURCE_DIR}/bin/${EXE_NAME}Staticp>
                        $<$<CONFIG:MinSizeRel>:${CMAKE_SOURCE_DIR}/bin/${EXE_NAME}Staticm> )
  endif()

  # set the folder where to place the projects
  set_target_properties( ${EXE_NAME}  PROPERTIES FOLDER app LINKER_LANGUAGE CXX )
endmacro()

define_DecoderXApp_target( DecoderApp "CommonLib;DecoderLib;Utilities" )

define_DecoderXApp_target( DecoderAnalyserApp "CommonAnalyserLib;DecoderAnalyserLib;Utilities" )
target_compile_definitions( DecoderAnalyserApp PUBLIC RExt__DECODER_DEBUG_BIT_STATISTICS=1 )
target_compile_definitions( DecoderAnalyserApp PUBLIC RExt__DECODER_DEBUG_TOOL_STATISTICS=1 )
