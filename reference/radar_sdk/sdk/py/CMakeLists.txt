
set(Python3_FIND_VIRTUALENV "ONLY")  # prefer Python virtual environment, if it's activated
find_package(Python3)

if(NOT ${Python3_FOUND})
    message("Python not found: Cannot build Python wrappers")
    return()
endif()

set(SDK_WHEEL_VERSION ${SDK_VERSION})

function(format_dependency_list IN OUT)
    list(TRANSFORM ${IN} PREPEND "\"$<TARGET_FILE_NAME:")
    list(TRANSFORM ${IN} APPEND ">\"")
    list(JOIN ${IN} ", " ${OUT})
    set(${OUT} ${${OUT}} PARENT_SCOPE)
endfunction()

strata_include(wheels)

add_subdirectory(wrapper_radarsdk)

