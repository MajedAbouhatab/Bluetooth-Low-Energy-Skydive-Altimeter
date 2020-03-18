#include <BDK.h>

#define BUFFER_SIZE 64*128/8

uint8_t oled_buffer[BUFFER_SIZE];

uint32_t TimeStamp;

const unsigned char Image_num_bmp[3 * BUFFER_SIZE];

extern void BitBanging_Init(void);

extern void WriteBufferToDisplay(void);

extern void SendAltitudeToDisplay(void);
