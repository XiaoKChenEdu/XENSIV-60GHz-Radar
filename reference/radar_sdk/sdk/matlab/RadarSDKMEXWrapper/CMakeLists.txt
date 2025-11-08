
set(CONTEXT_WRAPPER_SRC ../ContextWrapper/Octave5Compatibility.c ../ContextWrapper/MWrapHelpers.c ../ContextWrapper/MWrapInternalCommands.c)
matlab_add_mex(NAME DeviceControlM
               SRC DeviceControlM.c ${CONTEXT_WRAPPER_SRC}
               LINK_TO sdk_radar
               R2018a)

# add MATLAB folder containing lots of .dylib files that need to be linked
if(APPLE)
  set(MATLAB_BIN_ARCH_SUBFOLDER "maci64")

  if(CMAKE_OSX_ARCHITECTURES MATCHES "arm64")
    set(MATLAB_BIN_ARCH_SUBFOLDER "maca64")
  endif()

  target_link_directories(DeviceControlM PRIVATE ${Matlab_ROOT_DIR}/bin/${MATLAB_BIN_ARCH_SUBFOLDER})
endif()

set(ADDITIONAL_LIBS "")
if(WIN32 OR MINGW OR MSYS)
  find_program(MKOCTFILE_PATH NAMES mkoctfile.exe NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH)
  if(MKOCTFILE_PATH)
    set(MKOCTFILE_EXE ${MKOCTFILE_PATH})
  else()
    set(MKOCTFILE_EXE ${RDK_3RD_PARTY_DIR}/tools/octave/mingw64/bin/mkoctfile.exe)
  endif()
  set(ADDITIONAL_LIBS -lsetupapi -lws2_32) 
else() #UNIX
  find_program(MKOCTFILE_PATH NAMES mkoctfile NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH)
  set(MKOCTFILE_EXE ${MKOCTFILE_PATH})
endif()

if(EXISTS ${MKOCTFILE_EXE})
  set(MATLAB_CONTEXT_WRAPPER_DIR ${RDK_SDK_DIR}/matlab/ContextWrapper)
  set(RADAR_MEX_WRAPPER_DIR ${RDK_SDK_DIR}/matlab/RadarSDKMEXWrapper)

  add_custom_target(radar_mex ALL
      COMMAND echo "Building radar mex wrapper..."
  )

  add_dependencies(radar_mex sdk_radar)

  add_custom_command(TARGET radar_mex POST_BUILD
    WORKING_DIRECTORY ${RADAR_MEX_WRAPPER_DIR}
    COMMAND ${MKOCTFILE_EXE} ARGS --mex -DSTD_MEX -I${RDK_SDK_DIR}/c/ -L$<TARGET_FILE_DIR:sdk_radar> -l:$<TARGET_FILE_NAME:sdk_radar> ../ContextWrapper/MWrapInternalCommands.c ../ContextWrapper/MWrapHelpers.c DeviceControlM.c ${ADDITIONAL_LIBS} -o DeviceControlM.mex
    VERBATIM)
endif()
