idf_component_register(SRC_DIRS . param_test touch_sensor_test adc_dma_test dac_dma_test
                       PRIV_INCLUDE_DIRS include param_test/include touch_sensor_test/include
                       PRIV_REQUIRES cmock test_utils driver nvs_flash esp_serial_slave_link
                                     esp_adc_cal esp_timer)
