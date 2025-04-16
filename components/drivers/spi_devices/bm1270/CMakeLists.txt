idf_component_register(SRCS "bmi2.c" "bmi270_maximum_fifo.c" "bmi270.c" "common.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES crazyflie platform config spi_bus)