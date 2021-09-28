/*!
 * These displays use SPI to communicate, 4 or 5 pins are required
 * to interface (RST is optional).
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
 * Adafruit_GFX</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Limor "ladyada" Fried for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_GC9102_kbv.h"
#ifndef ARDUINO_STM32_FEATHER
#include "pins_arduino.h"
#ifndef RASPI
#include "wiring_private.h"
#endif
#endif
#include <limits.h>

#if defined(ARDUINO_ARCH_ARC32) || defined(ARDUINO_MAXIM)
#define SPI_DEFAULT_FREQ 16000000
// Teensy 3.0, 3.1/3.2, 3.5, 3.6
#elif defined(__MK20DX128__) || defined(__MK20DX256__) ||                      \
    defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define SPI_DEFAULT_FREQ 40000000
#elif defined(__AVR__) || defined(TEENSYDUINO)
#define SPI_DEFAULT_FREQ 8000000
#elif defined(ESP8266) || defined(ESP32)
#define SPI_DEFAULT_FREQ 40000000
#elif defined(RASPI)
#define SPI_DEFAULT_FREQ 80000000
#elif defined(ARDUINO_ARCH_STM32F1)
#define SPI_DEFAULT_FREQ 36000000
#else
#define SPI_DEFAULT_FREQ 24000000 ///< Default SPI data clock frequency
#endif

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit GC9102 driver with software SPI
    @param    cs    Chip select pin #
    @param    dc    Data/Command pin #
    @param    mosi  SPI MOSI pin #
    @param    sclk  SPI Clock pin #
    @param    rst   Reset pin # (optional, pass -1 if unused)
    @param    miso  SPI MISO pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
Adafruit_GC9102_kbv::Adafruit_GC9102_kbv(int8_t cs, int8_t dc, int8_t mosi,
                                   int8_t sclk, int8_t rst, int8_t miso)
    : Adafruit_SPITFT(GC9102_TFTWIDTH, GC9102_TFTHEIGHT, cs, dc, mosi, sclk,
                      rst, miso) {}

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit GC9102 driver with hardware SPI using the
            default SPI peripheral.
    @param  cs   Chip select pin # (OK to pass -1 if CS tied to GND).
    @param  dc   Data/Command pin # (required).
    @param  rst  Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
Adafruit_GC9102_kbv::Adafruit_GC9102_kbv(int8_t cs, int8_t dc, int8_t rst)
    : Adafruit_SPITFT(GC9102_TFTWIDTH, GC9102_TFTHEIGHT, cs, dc, rst) {}

#if !defined(ESP8266)
/**************************************************************************/
/*!
    @brief  Instantiate Adafruit GC9102 driver with hardware SPI using
            a specific SPI peripheral (not necessarily default).
    @param  spiClass  Pointer to SPI peripheral (e.g. &SPI or &SPI1).
    @param  dc        Data/Command pin # (required).
    @param  cs        Chip select pin # (optional, pass -1 if unused and
                      CS is tied to GND).
    @param  rst       Reset pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
Adafruit_GC9102_kbv::Adafruit_GC9102_kbv(SPIClass *spiClass, int8_t dc, int8_t cs,
                                   int8_t rst)
    : Adafruit_SPITFT(GC9102_TFTWIDTH, GC9102_TFTHEIGHT, spiClass, cs, dc,
                      rst) {}
#endif // end !ESP8266

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit GC9102 driver using parallel interface.
    @param  busWidth  If tft16 (enumeration in Adafruit_SPITFT.h), is a
                      16-bit interface, else 8-bit.
    @param  d0        Data pin 0 (MUST be a byte- or word-aligned LSB of a
                      PORT register -- pins 1-n are extrapolated from this).
    @param  wr        Write strobe pin # (required).
    @param  dc        Data/Command pin # (required).
    @param  cs        Chip select pin # (optional, pass -1 if unused and CS
                      is tied to GND).
    @param  rst       Reset pin # (optional, pass -1 if unused).
    @param  rd        Read strobe pin # (optional, pass -1 if unused).
*/
/**************************************************************************/
Adafruit_GC9102_kbv::Adafruit_GC9102_kbv(tftBusWidth busWidth, int8_t d0, int8_t wr,
                                   int8_t dc, int8_t cs, int8_t rst, int8_t rd)
    : Adafruit_SPITFT(GC9102_TFTWIDTH, GC9102_TFTHEIGHT, busWidth, d0, wr, dc,
                      cs, rst, rd) {}

// clang-format off
static const uint8_t PROGMEM initcmd[] = {
/* 2.2 CMO1.77 Init Code*/
    //  (COMMAND_BYTE), n, data_bytes....
    0x01, 0x80,         // Soft reset, then delay 150 ms
/*-----------------------display control setting------------------------------*/
 (0xfe), 0,
 (0xef), 0,
 (0x3a), 1,0x05,
 (0x36), 1,0xC0,
 (0xb4), 1,0x00,
/*---------------------------end display control setting-------------------------*/
/*-------------------------Power Control Registers Initial---------------------*/
 (0xa8), 3,0x02,0x00,0x00, //??
 (0xa7), 1,0x02,           //??
 (0xea), 1,0x3a,           //??
 (0xb4), 1,0x00,           //GC9102_INVCTL [07] ???
 (0xff), 1,0x13,           //GC9102_POWCTL4 [16]
 (0xfd), 1,0x10,           //GC9102_POWCTL3 [1C]
 (0xa4), 1,0x09,           //GC9102_POWCTL1 [16]
 (0xe7), 2,0x94,0x88,      //??
 (0xed), 1,0x11,           //GC9102_POWCTL2 [11 06]
 (0xe4), 1,0xc5,           //??
 (0xe2), 1,0x80,           //??
 (0xa3), 1,0x09,           //GC9102_FRAMERT [16 16 16]
 (0xe3), 1,0x07,           //??
 (0xe5), 1,0x10,           //??
/*-----------------------end Power Control Registers Initial--------------------*/
/*------------------------display window 128X160----------------------------*/
 (0x2a), 4,0x00,0x00,0x00,0x7f,
 (0x2b), 4,0x00,0x00,0x00,0x9f,
 (0x2c), 0,
/*-----------------------end display window 128X160-------------------------*/
/*------------------------------------gamma setting-------------------------------*/
 (0xf0), 1,0x03,
 (0xf1), 1,0x00,
 (0xf2), 1,0x00,
 (0xf3), 1,0x44,
 (0xf4), 1,0x00,
 (0xf5), 1,0x0c,
 (0xf7), 1,0x47,
 (0xf8), 1,0x00,
 (0xf9), 1,0x60,
 (0xfa), 1,0x22,
 (0xfb), 1,0x04,
 (0xfc), 1,0x00,
/*---------------------------------end gamma setting------------------------------*/
    0x11, 0x80, // Exit Sleep, then delay 150 ms
    0x29, 0x80, // Main screen turn on, delay 150 ms
  0x00                                   // End of list
};
// clang-format on

/**************************************************************************/
/*!
    @brief   Initialize GC9102 chip
    Connects to the GC9102 over SPI and sends initialization procedure commands
    @param    freq  Desired SPI clock frequency
*/
/**************************************************************************/
void Adafruit_GC9102_kbv::begin(uint32_t freq) {

  if (!freq)
    freq = SPI_DEFAULT_FREQ;
  initSPI(freq);

  if (_rst < 0) {                 // If no hardware reset pin...
    sendCommand(GC9102_SWRESET); // Engage software reset
    delay(150);
  }

  uint8_t cmd, x, numArgs;
  const uint8_t *addr = initcmd;
  while ((cmd = pgm_read_byte(addr++)) > 0) {
    x = pgm_read_byte(addr++);
    numArgs = x & 0x7F;
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;
    if (x & 0x80)
      delay(150);
  }

  _width = GC9102_TFTWIDTH;
  _height = GC9102_TFTHEIGHT;
}

/**************************************************************************/
/*!
    @brief   Set origin of (0,0) and orientation of TFT display
    @param   m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void Adafruit_GC9102_kbv::setRotation(uint8_t m) {
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
  case 0:
    m = (MADCTL_MX | MADCTL_ML | MADCTL_BGR);
    _width = GC9102_TFTWIDTH;
    _height = GC9102_TFTHEIGHT;
    break;
  case 1:
    m = (MADCTL_MV | MADCTL_ML | MADCTL_BGR);
    _width = GC9102_TFTHEIGHT;
    _height = GC9102_TFTWIDTH;
    break;
  case 2:
    m = (MADCTL_MY | MADCTL_BGR);
    _width = GC9102_TFTWIDTH;
    _height = GC9102_TFTHEIGHT;
    break;
  case 3:
    m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
    _width = GC9102_TFTHEIGHT;
    _height = GC9102_TFTWIDTH;
    break;
  }
  m ^= 0x80; //.kbv
  sendCommand(GC9102_MADCTL, &m, 1);
  setScrollMargins(0, 0); //.kbv
  scrollTo(0);   
}

/**************************************************************************/
/*!
    @brief   Enable/Disable display color inversion
    @param   invert True to invert, False to have normal color
*/
/**************************************************************************/
void Adafruit_GC9102_kbv::invertDisplay(bool invert) {
  sendCommand(invert ? GC9102_INVON : GC9102_INVOFF);
}

/**************************************************************************/
/*!
    @brief   Scroll display memory
    @param   y How many pixels to scroll display by
*/
/**************************************************************************/
void Adafruit_GC9102_kbv::scrollTo(uint16_t y) {
  uint8_t data[2];
  data[0] = y >> 8;
  data[1] = y & 0xff;
  sendCommand(GC9102_VSCRSADD, (uint8_t *)data, 2);
}

/**************************************************************************/
/*!
    @brief   Set the height of the Top and Bottom Scroll Margins
    @param   top The height of the Top scroll margin
    @param   bottom The height of the Bottom scroll margin
 */
/**************************************************************************/
void Adafruit_GC9102_kbv::setScrollMargins(uint16_t top, uint16_t bottom) {
  // TFA+VSA+BFA must equal 480
  if (top + bottom <= GC9102_TFTHEIGHT) {
    uint16_t middle = GC9102_TFTHEIGHT - top - bottom;
    uint8_t data[6];
    data[0] = top >> 8;
    data[1] = top & 0xff;
    data[2] = middle >> 8;
    data[3] = middle & 0xff;
    data[4] = bottom >> 8;
    data[5] = bottom & 0xff;
    sendCommand(GC9102_VSCRDEF, (uint8_t *)data, 6);
  }
}

/**************************************************************************/
/*!
    @brief   Set the "address window" - the rectangle we will write to RAM with
   the next chunk of      SPI data writes. The GC9102 will automatically wrap
   the data as each row is filled
    @param   x1  TFT memory 'x' origin
    @param   y1  TFT memory 'y' origin
    @param   w   Width of rectangle
    @param   h   Height of rectangle
*/
/**************************************************************************/
void Adafruit_GC9102_kbv::setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w,
                                     uint16_t h) {
  uint16_t x2 = (x1 + w - 1), y2 = (y1 + h - 1);
  writeCommand(GC9102_CASET); // Column address set
  SPI_WRITE16(x1);
  SPI_WRITE16(x2);
  writeCommand(GC9102_PASET); // Row address set
  SPI_WRITE16(y1);
  SPI_WRITE16(y2);
  writeCommand(GC9102_RAMWR); // Write to RAM
}


