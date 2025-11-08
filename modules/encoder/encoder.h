#include "bsp_spi.h"
#include <string.h>
#include <stdlib.h>

typedef struct EncoderInstance {
    int position;
    SPIInstance *encoder_spi_instance;
} EncoderInstance;

