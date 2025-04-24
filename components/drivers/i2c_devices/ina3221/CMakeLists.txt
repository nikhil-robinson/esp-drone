idf_component_register(SRCS "ina3221.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES i2c_bus)