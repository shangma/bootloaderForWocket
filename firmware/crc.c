/* Currently (Sept 2, 2011) the crc.c functions are not being used */

#define  POLYNOMIAL (0x1070 << 3)		// Polynomial CRC 

/* Function called by the computerCRC8 and CRC16 functions below*/
unsigned char ComputeByte(unsigned char inCrc, unsigned char inData)
{
    int i;
    unsigned short data;
	// Xor'ing inCrc with inData
    data = (unsigned short)(inCrc ^ inData);
    data <<= 8;
    for (i = 0; i < 8; i++)
   	{
        if ((data & 0x8000) != 0)
        	data = (unsigned short)(data ^ POLYNOMIAL);
    	data = (unsigned short)(data << 1);
    }
	return (unsigned char)(data >> 8);
}

/* Compute CRC for one byte*/ 
unsigned char ComputeCRC8(unsigned char crc, unsigned char *data, int len)
{
	for (int i = 0; (i < len); i++)
    	crc = ComputeByte(crc, *(data+i));
    return crc;
}

/* Compute CRC for two bytes of data or unsigned short*/   
unsigned short CRC16(unsigned char *buf, int len )
{
	unsigned short crc = 0;
	while( len-- ) {
		int i;
		crc ^= *(char *)buf++ << 8;
		for( i = 0; i < 8; ++i ) {
			if( crc & 0x8000 )
				crc = (crc << 1) ^ 0x1021;
			else
				crc = crc << 1;
		}
	}
	return crc;
}
