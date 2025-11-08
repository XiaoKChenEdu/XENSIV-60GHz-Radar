set(COMMON_SOURCES
    common.c
    json.cpp
    time_formatter.c
    util.c
)

set(COMMON_HEADERS
    common.h
    json.h
    json.hpp
    time_formatter.h
    util.h
)

add_library(examples_common STATIC ${COMMON_SOURCES} ${COMMON_HEADERS})
target_include_directories(examples_common PUBLIC .)
target_link_libraries(examples_common PUBLIC sdk_avian sdk_radar)
target_link_libraries(examples_common PRIVATE argparse nlohmann_json)
