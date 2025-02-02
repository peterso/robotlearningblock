file(GLOB_RECURSE C_SOURCES *.c **/*.c)
file(GLOB_RECURSE CPP_SOURCES *.cpp **/*.cpp)

idf_component_register(SRCS
        ${C_SOURCES}
        ${CPP_SOURCES}
        INCLUDE_DIRS ".")
