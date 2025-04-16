idf_component_register( SRCS "zranger2.c" "tof.c"
                        INCLUDE_DIRS "." "include" "core/inc"
                        REQUIRES i2c_bus crazyflie platform config)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-overflow")