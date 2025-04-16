idf_component_register(SRCS "spi_bus.c"
                       INCLUDE_DIRS "include"
                       REQUIRES crazyflie platform driver
                       PRIV_REQUIRES config)