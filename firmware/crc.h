/*Currently (Sept 2, 2011) the crc.h is not being used */
unsigned short crc=0;
unsigned short rcrc=0;
unsigned char ComputeCRC8(unsigned char crc, unsigned char *data, int len);
unsigned short CRC16(unsigned char *buf, int len );
