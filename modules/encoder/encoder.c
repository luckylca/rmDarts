#include <encoder.h>


EncoderInstance *EncoderInit(SPIInstance *spi_instance)
{
    EncoderInstance *encoder = malloc(sizeof(EncoderInstance));
    memset(encoder, 0, sizeof(EncoderInstance));

    encoder->position = 0;
    encoder->encoder_spi_instance = spi_instance;

    return encoder;
}
    