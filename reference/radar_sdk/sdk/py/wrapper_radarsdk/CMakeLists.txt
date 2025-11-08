
set(PACKAGE_NAME ifxradarsdk)
set(WHEEL_NAME ${PACKAGE_NAME})
set(WHEEL_DEPEDENCIES
    sdk_avian
    sdk_ltr11
    sdk_mimose
    sdk_cw
)

#string(REGEX MATCH "^([0-9]+\\.)+[0-9]+" WHEEL_VERSION ${rdk_VERSION})

# generate pyproject.toml from the template file
set(PYPROJECT_FILE pyproject.toml)
set(PYPROJECT_CONFIGURE_FILE pyproject.toml.in)
configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/${PYPROJECT_CONFIGURE_FILE}
    ${CMAKE_CURRENT_LIST_DIR}/${PYPROJECT_FILE}
    @ONLY
)

add_wheel(NAME ${WHEEL_NAME}
    DEPENDENCIES_DIR lib
    DEPENDENCIES ${WHEEL_DEPEDENCIES}
)
set_property(TARGET wheel-${WHEEL_NAME} APPEND PROPERTY SOURCES ${PYPROJECT_CONFIGURE_FILE})


set(GENERATE_EXCEPTIONS_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../generate_exceptions.py)

# workaround to add sources to custom targets on older CMake versions
set_property(TARGET wheel-${WHEEL_NAME} APPEND PROPERTY SOURCES
    ${GENERATE_EXCEPTIONS_SCRIPT}
)

add_custom_command(TARGET wheel-${WHEEL_NAME} PRE_BUILD
#    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/src/${PACKAGE_NAME}
    COMMAND ${Python3_EXECUTABLE} ${GENERATE_EXCEPTIONS_SCRIPT} -i ${RDK_SDK_DIR}/c/ifxBase/Error.h -o ${CMAKE_CURRENT_SOURCE_DIR}/src/${PACKAGE_NAME}/common/exceptions.py
)
