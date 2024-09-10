#include "CRC.h"

uint8_t crc8( uint8_t * p_buffer, uint16_t buf_size )
{
    uint8_t crc = 0;
	uint8_t i=0;
    if(buf_size <= 0)
    {
        return crc;
    }
    while( buf_size-- )
    {
        for (  i = 0x80; i != 0; i /= 2 )
        {
            if ( (crc & 0x80) != 0)
            {
                crc *= 2;
                crc ^= 0x07; 
            }
            else
            {
                crc *= 2;
            }
 
            if ( (*p_buffer & i) != 0 )
            {
                crc ^= 0x07;
            }
        }
        p_buffer++;
    }
    return crc;
}
