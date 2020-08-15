#define ARRAY_NULL 0


/*!
 * Combo ported library from adafruit's arduino SSD1306 and GFX libraries.
 */

#ifndef _Adafruit_SSD1306_H_
#define _Adafruit_SSD1306_H_

#include </usr/include/stdint.h>
#include </usr/include/linux/types.h>
#include </usr/include/linux/stat.h>
#include </usr/include/string.h>
#include </usr/include/stdlib.h>
#include </usr/include/stdio.h>
#include </usr/include/linux/ioctl.h>
#include </usr/include/linux/i2c.h>
#include </usr/include/linux/i2c-dev.h>
#include </usr/include/errno.h>

#include </usr/include/arm-linux-gnueabihf/sys/ioctl.h>
#include </usr/include/fcntl.h>
#include </usr/include/unistd.h>
#include </usr/include/linux/ioctl.h>
#include </usr/include/linux/i2c.h>
#include </usr/include/linux/i2c-dev.h>

#include "gldcfont.c"
#include "splash.h"

#include "ros/ros.h"
#include "arduinoPort.h"
#include "arduinoPort.cpp"

using namespace std;

#define WIDTH 128 // OLED display width, in pixels
#define HEIGHT 64 // OLED display height, in pixels

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

/// The following "raw" color names are kept for backwards client compatability
/// They can be disabled by predefining this macro before including the Adafruit header
/// client code will then need to be modified to use the scoped enum values directly
#ifndef NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY
#define BLACK                     SSD1306_BLACK    ///< Draw 'off' pixels
#define WHITE                     SSD1306_WHITE    ///< Draw 'on' pixels
#define INVERSE                   SSD1306_INVERSE  ///< Invert pixels
#endif
        /// fit into the SSD1306_ naming scheme
#define SSD1306_BLACK               0    ///< Draw 'off' pixels
#define SSD1306_WHITE               1    ///< Draw 'on' pixels
#define SSD1306_INVERSE             2    ///< Invert pixels

#define SSD1306_MEMORYMODE          0x20 ///< See datasheet
#define SSD1306_COLUMNADDR          0x21 ///< See datasheet
#define SSD1306_PAGEADDR            0x22 ///< See datasheet
#define SSD1306_SETCONTRAST         0x81 ///< See datasheet
#define SSD1306_CHARGEPUMP          0x8D ///< See datasheet
#define SSD1306_SEGREMAP            0xA0 ///< See datasheet
#define SSD1306_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SSD1306_DISPLAYALLON        0xA5 ///< Not currently used
#define SSD1306_NORMALDISPLAY       0xA6 ///< See datasheet
#define SSD1306_INVERTDISPLAY       0xA7 ///< See datasheet
#define SSD1306_SETMULTIPLEX        0xA8 ///< See datasheet
#define SSD1306_DISPLAYOFF          0xAE ///< See datasheet
#define SSD1306_DISPLAYON           0xAF ///< See datasheet
#define SSD1306_COMSCANINC          0xC0 ///< Not currently used
#define SSD1306_COMSCANDEC          0xC8 ///< See datasheet
#define SSD1306_SETDISPLAYOFFSET    0xD3 ///< See datasheet
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5 ///< See datasheet
#define SSD1306_SETPRECHARGE        0xD9 ///< See datasheet
#define SSD1306_SETCOMPINS          0xDA ///< See datasheet
#define SSD1306_SETVCOMDETECT       0xDB ///< See datasheet

#define SSD1306_SETLOWCOLUMN        0x00 ///< Not currently used
#define SSD1306_SETHIGHCOLUMN       0x10 ///< Not currently used
#define SSD1306_SETSTARTLINE        0x40 ///< See datasheet

#define SSD1306_EXTERNALVCC         0x01 ///< External display voltage source
#define SSD1306_SWITCHCAPVCC        0x02 ///< Gen. display voltage from 3.3V

#define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26 ///< Init rt scroll
#define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27 ///< Init left scroll
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29 ///< Init diag scroll
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A ///< Init diag scroll
#define SSD1306_DEACTIVATE_SCROLL                    0x2E ///< Stop scroll
#define SSD1306_ACTIVATE_SCROLL                      0x2F ///< Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3 ///< Set scroll range


/*! 
    @brief  Class that stores state and functions for interacting with
            SSD1306 OLED displays.
*/
//class Adafruit_SSD1306 : public Adafruit_GFX {
class Adafruit_SSD1306 {
 public:
  // NEW CONSTRUCTORS -- recommended for new projects
  Adafruit_SSD1306(uint8_t w, uint8_t h);

  ~Adafruit_SSD1306(void);
  bool      begin(uint8_t switchvcc=SSD1306_SWITCHCAPVCC,
                 uint8_t i2caddr=0, bool reset=true,
                 bool periphBegin=true);
  void         display(void);
  void         clearDisplay();
  void         invertDisplay(bool i);
  void         dim(bool dim);
  void         drawPixel(int16_t x, int16_t y, uint16_t color);
  void         drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                              int16_t w, int16_t h, uint16_t color);
  size_t       write(uint8_t cursor_x, uint8_t cursor_y, 
                            string message, bool invertColor, bool wrapRight, 
                            uint8_t size);
  void         drawChar(int16_t x, int16_t y, unsigned char c,
                            uint16_t color, uint16_t bg, uint8_t size_x,
                            uint8_t size_y);
  void         fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                            uint16_t color);
  void         drawFastVLineInternal(
                    int16_t x, int16_t __y, int16_t __h, uint16_t color);
  void         drawFastHLineInternal(
                        int16_t x, int16_t y, int16_t w, uint16_t color);
  size_t       writeError(uint8_t cursor_x, uint8_t cursor_y, 
                            string message, char noError, bool invertColor, bool wrapRight, 
                            uint8_t size);
  //virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  //virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  //void         startscrollright(uint8_t start, uint8_t stop);
  //void         startscrollleft(uint8_t start, uint8_t stop);
  //void         startscrolldiagright(uint8_t start, uint8_t stop);
  //void         startscrolldiagleft(uint8_t start, uint8_t stop);
  //void         stopscroll(void);
  void         ssd1306_command(uint8_t c);
  bool      getPixel(int16_t x, int16_t y);
  uint8_t     *getBuffer(void);
  uint8_t     *buffer;

 private:
  //void         drawFastHLineInternal(int16_t x, int16_t y, int16_t w,
  //               uint16_t color);
  //void         drawFastVLineInternal(int16_t x, int16_t y, int16_t h,
  //               uint16_t color);
  void         ssd1306_command1(uint8_t c);
  void         ssd1306_commandList(const uint8_t *c, uint8_t n);

  int8_t       i2caddr, vccstate, page_end;
};

#endif // _Adafruit_SSD1306_H_
