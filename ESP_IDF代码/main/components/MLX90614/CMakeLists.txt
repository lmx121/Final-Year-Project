idf_component_register(SRCS "MLX90614_SMBus_Driver.c" 
                       SRCS "MLX90614_API.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver spi_flash)
