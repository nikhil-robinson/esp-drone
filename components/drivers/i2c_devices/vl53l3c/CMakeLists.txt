idf_component_register(SRCS "zranger2.c" "vl53lx_api_calibration.c" "vl53lx_api_core.c" "vl53lx_api_debug.c" "vl53lx_api_preset_modes.c" "vl53lx_api.c" "vl53lx_core_support.c" "vl53lx_core.c" "vl53lx_dmax.c" "vl53lx_hist_algos_gen3.c" "vl53lx_hist_algos_gen4.c" "vl53lx_hist_char.c" "vl53lx_hist_core.c" "vl53lx_hist_funcs.c" "vl53lx_nvm_debug.c" "vl53lx_nvm.c" "vl53lx_platform_init.c" "vl53lx_platform_ipp.c" "vl53lx_platform_log.c" "vl53lx_platform.c" "vl53lx_register_funcs.c" "vl53lx_sigma_estimate.c" "vl53lx_silicon_core.c" "vl53lx_wait.c" "vl53lx_xtalk.c"
                      INCLUDE_DIRS "." "include"
                      REQUIRES i2c_bus crazyflie platform config)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-overflow")