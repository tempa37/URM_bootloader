#include <stdint.h>
#include <stddef.h>

uint16_t mbcrc(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--)
    {
        crc ^= *data++;

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;   // Modbus polynomial (reflected)
            else
                crc >>= 1;
        }
    }

    return (uint16_t)((crc << 8) | (crc >> 8));
}