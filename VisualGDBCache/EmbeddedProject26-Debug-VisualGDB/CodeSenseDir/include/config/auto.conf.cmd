deps_config := \
	/esp-idf/v3.2/components/app_trace/Kconfig \
	/esp-idf/v3.2/components/aws_iot/Kconfig \
	/esp-idf/v3.2/components/bt/Kconfig \
	/esp-idf/v3.2/components/driver/Kconfig \
	/esp-idf/v3.2/components/esp32/Kconfig \
	/esp-idf/v3.2/components/esp_adc_cal/Kconfig \
	/esp-idf/v3.2/components/esp_event/Kconfig \
	/esp-idf/v3.2/components/esp_http_client/Kconfig \
	/esp-idf/v3.2/components/esp_http_server/Kconfig \
	/esp-idf/v3.2/components/ethernet/Kconfig \
	/esp-idf/v3.2/components/fatfs/Kconfig \
	/esp-idf/v3.2/components/freemodbus/Kconfig \
	/esp-idf/v3.2/components/freertos/Kconfig \
	/esp-idf/v3.2/components/heap/Kconfig \
	/esp-idf/v3.2/components/libsodium/Kconfig \
	/esp-idf/v3.2/components/log/Kconfig \
	/esp-idf/v3.2/components/lwip/Kconfig \
	/esp-idf/v3.2/components/mbedtls/Kconfig \
	/esp-idf/v3.2/components/mdns/Kconfig \
	/esp-idf/v3.2/components/mqtt/Kconfig \
	/esp-idf/v3.2/components/nvs_flash/Kconfig \
	/esp-idf/v3.2/components/openssl/Kconfig \
	/esp-idf/v3.2/components/pthread/Kconfig \
	/esp-idf/v3.2/components/spi_flash/Kconfig \
	/esp-idf/v3.2/components/spiffs/Kconfig \
	/esp-idf/v3.2/components/tcpip_adapter/Kconfig \
	/esp-idf/v3.2/components/vfs/Kconfig \
	/esp-idf/v3.2/components/wear_levelling/Kconfig \
	/esp-idf/v3.2/components/bootloader/Kconfig.projbuild \
	/esp-idf/v3.2/components/esptool_py/Kconfig.projbuild \
	/esp-idf/v3.2/components/partition_table/Kconfig.projbuild \
	/esp-idf/v3.2/Kconfig

include/config/auto.conf: \
	$(deps_config)

ifneq "$(IDF_CMAKE)" "n"
include/config/auto.conf: FORCE
endif

$(deps_config): ;
