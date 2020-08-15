/*!
 * Used to drive monochrome I2C SSD1306 OLED displays.
 * This is a ported library from the Adafruit's Arduino SSD1306 and GFX libraries, 
 * with some new features added that are specific to the application. 
 * 
 * Original libraries info:
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 */

#include "SSD1306_GFX.h"

// SOME DEFINES AND STATIC VARIABLES USED INTERNALLY -----------------------

#define ssd1306_swap(a, b) \
  (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation

 #define WIRE_WRITE Wirewrite// Write data function. Should write to a buffer.



// 'Transaction' start/end for I2C only changes clock speed
#define TRANSACTION_START(a) WirebeginTransmission() // Defines I2C clock via settings. Use for higher transmission speed.
#define TRANSACTION_END WireendTransmission // Resets I2C clock via library settings. Use for higher transmission speed.

// CONSTRUCTORS, DESTRUCTOR ------------------------------------------------

/*!
    @brief  Constructor for I2C-interfaced SSD1306 displays.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  twi
            Pointer to an existing TwoWire instance (e.g. &Wire, the
            microcontroller's primary I2C bus).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  clkDuring
            Speed (in Hz) for Wire transmissions in SSD1306 library calls.
            Defaults to 400000 (400 KHz), a known 'safe' value for most
            microcontrollers, and meets the SSD1306 datasheet spec.
            Some systems can operate I2C faster (800 KHz for ESP32, 1 MHz
            for many other 32-bit MCUs), and some (perhaps not all)
            SSD1306's can work with this -- so it's optionally be specified
            here and is not a default behavior. (Ignored if using pre-1.5.7
            Arduino software, which operates I2C at a fixed 100 KHz.)
    @param  clkAfter
            Speed (in Hz) for Wire transmissions following SSD1306 library
            calls. Defaults to 100000 (100 KHz), the default Arduino Wire
            speed. This is done rather than leaving it at the 'during' speed
            because other devices on the I2C bus might not be compatible
            with the faster rate. (Ignored if using pre-1.5.7 Arduino
            software, which operates I2C at a fixed 100 KHz.)
    @return Adafruit_SSD1306 object.
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
Adafruit_SSD1306::Adafruit_SSD1306(uint8_t w, uint8_t h)
{
}


/*!
    @brief  Destructor for Adafruit_SSD1306 object.
*/
Adafruit_SSD1306::~Adafruit_SSD1306(void) {
  if(buffer) {
    //free(buffer);
  memset(buffer, 0, WIDTH * ((HEIGHT + 7) / 8));
  }
  
}

// I2C command sending -----------------------------------------------
// Issue single command to SSD1306, using I2C or hard/soft SPI as needed.
// Because command calls are often grouped, SPI transaction and selection
// must be started/ended in calling function for efficiency.
// This is a private function, not exposed (see ssd1306_command() instead).
void Adafruit_SSD1306::ssd1306_command1(uint8_t c) {
    TRANSACTION_END();
    TRANSACTION_START(i2caddr);
    WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
    WIRE_WRITE(c);
    TRANSACTION_END();
}

// Issue list of commands to SSD1306, same rules as above re: transactions.
// This is a private function, not exposed.
void Adafruit_SSD1306::ssd1306_commandList(const uint8_t *c, uint8_t n) {
    TRANSACTION_START(i2caddr);
    WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
    uint8_t bytesOut = 1;
    while(n--) {
      if(bytesOut >= I2C_BUFFER_LENGTH) {
        TRANSACTION_END();
        TRANSACTION_START(i2caddr);
        WIRE_WRITE((uint8_t)0x00); // Co = 0, D/C = 0
        bytesOut = 1;
      }
      WIRE_WRITE(pgm_read_byte(c++));
      bytesOut++;
    }
    TRANSACTION_END();
}

// A public version of ssd1306_command1(), for existing user code that
// might rely on that function. This encapsulates the command transfer
// in a transaction start/end, similar to old library's handling of it.
/*!
    @brief  Issue a single low-level command directly to the SSD1306
            display, bypassing the library.
    @param  c
            Command to issue (0x00 to 0xFF, see datasheet).
    @return None (void).
*/
void Adafruit_SSD1306::ssd1306_command(uint8_t c) {
  TRANSACTION_START(i2caddr);
  ssd1306_command1(c);
  TRANSACTION_END();
}


// ALLOCATE & INIT DISPLAY -------------------------------------------------

/*!
    @brief  Allocate RAM for image buffer, initialize peripherals and pins.
    @param  vcs
            VCC selection. Pass SSD1306_SWITCHCAPVCC to generate the display
            voltage (step up) from the 3.3V source, or SSD1306_EXTERNALVCC
            otherwise. Most situations with Adafruit SSD1306 breakouts will
            want SSD1306_SWITCHCAPVCC.
    @param  addr
            I2C address of corresponding SSD1306 display (or pass 0 to use
            default of 0x3C for 128x32 display, 0x3D for all others).
            SPI displays (hardware or software) do not use addresses, but
            this argument is still required (pass 0 or any value really,
            it will simply be ignored). Default if unspecified is 0.
    @param  reset
            If true, and if the reset pin passed to the constructor is
            valid, a hard reset will be performed before initializing the
            display. If using multiple SSD1306 displays on the same bus, and
            if they all share the same reset pin, you should only pass true
            on the first display being initialized, false on all others,
            else the already-initialized displays would be reset. Default if
            unspecified is true.
    @param  periphBegin
            If true, and if a hardware peripheral is being used (I2C or SPI,
            but not software SPI), call that peripheral's begin() function,
            else (false) it has already been done in one's sketch code.
            Cases where false might be used include multiple displays or
            other devices sharing a common bus, or situations on some
            platforms where a nonstandard begin() function is available
            (e.g. a TwoWire interface on non-default pins, as can be done
            on the ESP8266 and perhaps others).
    @return true on successful allocation/init, false otherwise.
            Well-behaved code should check the return value before
            proceeding.
    @note   MUST call this function before any drawing or updates!
*/
bool Adafruit_SSD1306::begin(uint8_t vcs, uint8_t addr, bool reset,
  bool periphBegin) {

  if((!buffer) && !(buffer = (uint8_t *)malloc(WIDTH * ((HEIGHT + 7) / 8))))
    return false;

  clearDisplay();
//  if(HEIGHT > 32) {
 //   drawBitmap((WIDTH - splash1_width) / 2, (HEIGHT - splash1_height) / 2,
 //     splash1_data, splash1_width, splash1_height, 1);
 // } else {
 //   drawBitmap((WIDTH - splash2_width) / 2, (HEIGHT - splash2_height) / 2,
 //     splash2_data, splash2_width, splash2_height, 1);
 // }

  vccstate = vcs;


    i2caddr = addr;

  Wirebegin(i2caddr);
  TRANSACTION_START(i2caddr);

  // Init sequence
  static const uint8_t init1[] = {
    SSD1306_DISPLAYOFF,                   // 0xAE
    SSD1306_SETDISPLAYCLOCKDIV,           // 0xD5
    0x80,                                 // the suggested ratio 0x80
    SSD1306_SETMULTIPLEX };               // 0xA8
  ssd1306_commandList(init1, sizeof(init1));
  ssd1306_command1(HEIGHT - 1);

  static const uint8_t init2[] = {
    SSD1306_SETDISPLAYOFFSET,             // 0xD3
    0x0,                                  // no offset
    SSD1306_SETSTARTLINE | 0x0,           // line #0
    SSD1306_CHARGEPUMP };                 // 0x8D
  ssd1306_commandList(init2, sizeof(init2));

  ssd1306_command1((vccstate == SSD1306_EXTERNALVCC) ? 0x10 : 0x14);

  static const uint8_t init3[] = {
    SSD1306_MEMORYMODE,                   // 0x20
    0x00,                                 // 0x0 act like ks0108
    SSD1306_SEGREMAP | 0x1,
    SSD1306_COMSCANDEC };
  ssd1306_commandList(init3, sizeof(init3));

  if((WIDTH == 128) && (HEIGHT == 32)) {
    static const uint8_t init4a[] = {
      SSD1306_SETCOMPINS,                 // 0xDA
      0x02,
      SSD1306_SETCONTRAST,                // 0x81
      0x8F };
    ssd1306_commandList(init4a, sizeof(init4a));
  } else if((WIDTH == 128) && (HEIGHT == 64)) {
    static const uint8_t init4b[] = {
      SSD1306_SETCOMPINS,                 // 0xDA
      0x12,
      SSD1306_SETCONTRAST };              // 0x81
    ssd1306_commandList(init4b, sizeof(init4b));
    ssd1306_command1((vccstate == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF);
  } else if((WIDTH == 96) && (HEIGHT == 16)) {
    static const uint8_t init4c[] = {
      SSD1306_SETCOMPINS,                 // 0xDA
      0x2,    // ada x12
      SSD1306_SETCONTRAST };              // 0x81
    ssd1306_commandList(init4c, sizeof(init4c));
    ssd1306_command1((vccstate == SSD1306_EXTERNALVCC) ? 0x10 : 0xAF);
  } else {
    // Other screen varieties -- TBD
  }

  ssd1306_command1(SSD1306_SETPRECHARGE); // 0xd9
  ssd1306_command1((vccstate == SSD1306_EXTERNALVCC) ? 0x22 : 0xF1);
  static const uint8_t init5[] = {
    SSD1306_SETVCOMDETECT,               // 0xDB
    0x40,
    SSD1306_DISPLAYALLON_RESUME,         // 0xA4
    SSD1306_NORMALDISPLAY,               // 0xA6
    SSD1306_DEACTIVATE_SCROLL,
    SSD1306_DISPLAYON };                 // Main screen turn on
  ssd1306_commandList(init5, sizeof(init5));

  TRANSACTION_END();

  return true; // Success
}

// DRAWING FUNCTIONS -------------------------------------------------------

/*!
    @brief  Set/clear/invert a single pixel. This is also invoked by the
            Adafruit_GFX library in generating many higher-level graphics
            primitives.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @param  color
            Pixel color, one of: SSD1306_BLACK, SSD1306_WHITE or SSD1306_INVERT.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void Adafruit_SSD1306::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if((x >= 0) && (x < WIDTH) && (y >= 0) && (y < HEIGHT)) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    //switch(getRotation()) {
     //case 1:
      //ssd1306_swap(x, y);
      //x = WIDTH - x - 1;
      //break;
     //case 2:
      //x = WIDTH  - x - 1;
      //y = HEIGHT - y - 1;
      //break;
     //case 3:
      //ssd1306_swap(x, y);
      //y = HEIGHT - y - 1;
      //break;
    //}
    switch(color) {
     case SSD1306_WHITE:   buffer[x + (y/8)*WIDTH] |=  (1 << (y&7)); break;
     case SSD1306_BLACK:   buffer[x + (y/8)*WIDTH] &= ~(1 << (y&7)); break;
     case SSD1306_INVERSE: buffer[x + (y/8)*WIDTH] ^=  (1 << (y&7)); break;
    }
  }
}

/*!
    @brief  Clear contents of display buffer (set all pixels to off).
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void Adafruit_SSD1306::clearDisplay() {
  memset(buffer, ARRAY_NULL, WIDTH * ((HEIGHT + 7) / 8));
}


/*!
    @brief  Return color of a single pixel in display buffer.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @return true if pixel is set (usually SSD1306_WHITE, unless display invert mode
            is enabled), false if clear (SSD1306_BLACK).
    @note   Reads from buffer contents; may not reflect current contents of
            screen if display() has not been called.
*/
bool Adafruit_SSD1306::getPixel(int16_t x, int16_t y) {
  if((x >= 0) && (x < WIDTH) && (y >= 0) && (y < HEIGHT)) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    //switch(getRotation()) {
     //case 1:
      //ssd1306_swap(x, y);
      //x = WIDTH - x - 1;
      //break;
     //case 2:
      //x = WIDTH  - x - 1;
      //y = HEIGHT - y - 1;
      //break;
     //case 3:
      //ssd1306_swap(x, y);
      //y = HEIGHT - y - 1;
      //break;
    //}
    return (buffer[x + (y / 8) * WIDTH] & (1 << (y & 7)));
  }
  return false; // Pixel out of bounds
}

/*!
    @brief  Get base address of display buffer for direct reading or writing.
    @return Pointer to an unsigned 8-bit array, column-major, columns padded
            to full byte boundary if needed.
*/
uint8_t *Adafruit_SSD1306::getBuffer(void) {
  return buffer;
}

// REFRESH DISPLAY ---------------------------------------------------------

/*!
    @brief  Push data currently in RAM to SSD1306 display.
    @return None (void).
    @note   Drawing operations are not visible until this function is
            called. Call after each graphics command, or after a whole set
            of graphics commands, as best needed by one's own application.
*/
void Adafruit_SSD1306::display(void) {
  TRANSACTION_START(i2caddr);
  static const uint8_t dlist1[] = {
    SSD1306_PAGEADDR,
    0,                         // Page start address
    0xFF,                      // Page end (not really, but works here)
    SSD1306_COLUMNADDR,
    0 };                       // Column start address
  ssd1306_commandList(dlist1, sizeof(dlist1));
  ssd1306_command1(WIDTH - 1); // Column end address
  
  uint16_t count = WIDTH * ((HEIGHT + 7) / 8);
  uint8_t *ptr   = buffer;
    WirebeginTransmission();
    WIRE_WRITE((uint8_t)0x40);
    uint8_t bytesOut = 1;
    while(count--) {
      if(bytesOut >= I2C_BUFFER_LENGTH) {
        WireendTransmission();
        WirebeginTransmission();
        WIRE_WRITE((uint8_t)0x40);
        bytesOut = 1;
      }
      WIRE_WRITE(*ptr++);
      bytesOut++;
    }
    WireendTransmission();
}


// OTHER HARDWARE SETTINGS -------------------------------------------------

/*!
    @brief  Enable or disable display invert mode (white-on-black vs
            black-on-white).
    @param  i
            If true, switch to invert mode (black-on-white), else normal
            mode (white-on-black).
    @return None (void).
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed, rather a
            different pixel mode of the display hardware is used. When
            enabled, drawing SSD1306_BLACK (value 0) pixels will actually draw white,
            SSD1306_WHITE (value 1) will draw black.
*/
void Adafruit_SSD1306::invertDisplay(bool i) {
  TRANSACTION_START(i2caddr);
  ssd1306_command1(i ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
  TRANSACTION_END();
}

/*!
    @brief  Dim the display.
    @param  dim
            true to enable lower brightness mode, false for full brightness.
    @return None (void).
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed.
*/
void Adafruit_SSD1306::dim(bool dim) {
  uint8_t contrast;

  if(dim) {
    contrast = 0; // Dimmed display
  } else {
    contrast = (vccstate == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF;
  }
  // the range of contrast to too small to be really useful
  // it is useful to dim the display
  TRANSACTION_START(i2caddr);
  ssd1306_command1(SSD1306_SETCONTRAST);
  ssd1306_command1(contrast);
  TRANSACTION_END();
}

// GFX FROM ADAFRUIT_GFX LIBRARY
/**************************************************************************/
/*!
   @brief      Draw a PROGMEM-resident 1-bit image at the specified (x,y)
   position, using the specified foreground color (unset bits are transparent).
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void Adafruit_SSD1306::drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                              int16_t w, int16_t h, uint16_t color) {

  int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
  uint8_t byte = 0;

  //startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if (i & 7)
        byte <<= 1;
      else
        byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
      if (byte & 0x80)
        drawPixel(x + i, y, color);
    }
  }
  //endWrite();
}

size_t Adafruit_SSD1306::write(uint8_t cursor_x, uint8_t cursor_y, 
                            string message, bool invertColor = false, bool wrapRight = false, 
                            uint8_t size = 1) {
    uint8_t textsize_y = size; // Normally these are variables in the full Adafruit GFX library
    uint8_t textsize_x = size; // These are text size scalars.
    #define textcolor (invertColor ? SSD1306_BLACK : SSD1306_WHITE)
    #define textbgcolor (!invertColor ? SSD1306_BLACK : SSD1306_WHITE)
    uint8_t count = 0;
    if(wrapRight) cursor_x = 127 - (textsize_x * 6 * message.length());
    while(count < message.length()){
        if (message.at(count) != '\r' && message.at(count) != '\n') {
            drawChar(cursor_x, cursor_y, message.at(count), textcolor, textbgcolor, textsize_x,
                        textsize_y);
            if(message.at(count) != ' ')
                cursor_x += textsize_x * 6; // Advance x one char
            else
                cursor_x += textsize_x * 3; // Advance space one half char
        }
        count++;
    }
    return 1;
}

// Writes an error to the screen where any character that isn't 'noError'
//  is written in a different color
size_t Adafruit_SSD1306::writeError(uint8_t cursor_x, uint8_t cursor_y, 
                            string message, char noError, bool invertColor = false, bool wrapRight = false, 
                            uint8_t size = 1) {
    uint8_t textsize_y = size; // Normally these are variables in the full Adafruit GFX library
    uint8_t textsize_x = size; // These are text size scalars.
    #define textcolor (invertColor ? SSD1306_BLACK : SSD1306_WHITE)
    #define textbgcolor (!invertColor ? SSD1306_BLACK : SSD1306_WHITE)
    uint8_t count = 0;
    if(wrapRight) cursor_x = 127 - (textsize_x * 6 * message.length());
    while(count < message.length()){
        if (message.at(count) != '\r' && message.at(count) != '\n') {
            drawChar(cursor_x, cursor_y, message.at(count), 
                    (message.at(count) == noError)?textcolor:textbgcolor,
                    (message.at(count) == noError)?textbgcolor:textcolor,
                    textsize_x, textsize_y);
            if(message.at(count) != ' ')
                cursor_x += textsize_x * 6; // Advance x one char
            else
                cursor_x += textsize_x * 3; // Advance space one half char
        }
        count++;
    }
    return 1;
}

void Adafruit_SSD1306::drawChar(int16_t x, int16_t y, unsigned char c,
                            uint16_t color, uint16_t bg, uint8_t size_x,
                            uint8_t size_y) {
    if ((x >= WIDTH) ||              // Clip right
        (y >= HEIGHT) ||             // Clip bottom
        ((x + 6 * size_x - 1) < 0) || // Clip left
        ((y + 8 * size_y - 1) < 0))   // Clip top
      return;
      
    for (int8_t i = 0; i < 5; i++) { // Char bitmap = 5 columns
      uint8_t line = pgm_read_byte(&font[c * 5 + i]);
      for (int8_t j = 0; j < 8; j++, line >>= 1) { // Cycle each bit in the line
        if (line & 1) { // Check lsb in variable
          if (size_x == 1 && size_y == 1)
            drawPixel(x + i, y + j, color);
          else
            fillRect(x + i * size_x, y + j * size_y, size_x, size_y,
                          color);
        } else if (bg != color) {
          if (size_x == 1 && size_y == 1)
            drawPixel(x + i, y + j, bg);
          else
            fillRect(x + i * size_x, y + j * size_y, size_x, size_y, bg);
        }
      }
    }
    drawFastHLineInternal(x, y - 1, 5 * size_x, bg);
    // This made it harder to read close indiviual characters of different colors.
    //if (bg != color) { // If opaque, draw vertical line for last column
      //if (size_x == 1 && size_y == 1)
    drawFastVLineInternal(x - 1, y - 1, (8 * size_y) + 1, bg);
      //else
        //fillRect(x + 5 * size_x, y, size_x, 8 * size_y, bg);
    //}
}

void Adafruit_SSD1306::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                            uint16_t color) {
  for (int16_t i = x; i < x + w; i++) {
    drawFastVLineInternal(i, y, h, color);
  }
}

void Adafruit_SSD1306::drawFastVLineInternal(
  int16_t x, int16_t __y, int16_t __h, uint16_t color) {

  if((x >= 0) && (x < WIDTH)) { // X coord in bounds?
    if(__y < 0) { // Clip top
      __h += __y;
      __y = 0;
    }
    if((__y + __h) > HEIGHT) { // Clip bottom
      __h = (HEIGHT - __y);
    }
    if(__h > 0) { // Proceed only if height is now positive
      // this display doesn't need ints for coordinates,
      // use local byte registers for faster juggling
      uint8_t  y = __y, h = __h;
      uint8_t *pBuf = &buffer[(y / 8) * WIDTH + x];

      // do the first partial byte, if necessary - this requires some masking
      uint8_t mod = (y & 7);
      if(mod) {
        // mask off the high n bits we want to set
        mod = 8 - mod;
        // note - lookup table results in a nearly 10% performance
        // improvement in fill* functions
        // uint8_t mask = ~(0xFF >> mod);
        static const uint8_t premask[8] =
          { 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        uint8_t mask = pgm_read_byte(&premask[mod]);
        // adjust the mask if we're not going to reach the end of this byte
        if(h < mod) mask &= (0XFF >> (mod - h));

        switch(color) {
         case SSD1306_WHITE:   *pBuf |=  mask; break;
         case SSD1306_BLACK:   *pBuf &= ~mask; break;
         case SSD1306_INVERSE: *pBuf ^=  mask; break;
        }
        pBuf += WIDTH;
      }

      if(h >= mod) { // More to go?
        h -= mod;
        // Write solid bytes while we can - effectively 8 rows at a time
        if(h >= 8) {
          if(color == SSD1306_INVERSE) {
            // separate copy of the code so we don't impact performance of
            // black/white write version with an extra comparison per loop
            do {
              *pBuf ^= 0xFF;  // Invert byte
              pBuf  += WIDTH; // Advance pointer 8 rows
              h     -= 8;     // Subtract 8 rows from height
            } while(h >= 8);
          } else {
            // store a local value to work with
            uint8_t val = (color != SSD1306_BLACK) ? 255 : 0;
            do {
              *pBuf = val;    // Set byte
              pBuf += WIDTH;  // Advance pointer 8 rows
              h    -= 8;      // Subtract 8 rows from height
            } while(h >= 8);
          }
        }

        if(h) { // Do the final partial byte, if necessary
          mod = h & 7;
          // this time we want to mask the low bits of the byte,
          // vs the high bits we did above
          // uint8_t mask = (1 << mod) - 1;
          // note - lookup table results in a nearly 10% performance
          // improvement in fill* functions
          static const uint8_t PROGMEM postmask[8] =
            { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
          uint8_t mask = pgm_read_byte(&postmask[mod]);
          switch(color) {
           case SSD1306_WHITE:   *pBuf |=  mask; break;
           case SSD1306_BLACK:   *pBuf &= ~mask; break;
           case SSD1306_INVERSE: *pBuf ^=  mask; break;
          }
        }
      }
    } // endif positive height
  } // endif x in bounds
}

void Adafruit_SSD1306::drawFastHLineInternal(
  int16_t x, int16_t y, int16_t w, uint16_t color) {

  if((y >= 0) && (y < HEIGHT)) { // Y coord in bounds?
    if(x < 0) { // Clip left
      w += x;
      x  = 0;
    }
    if((x + w) > WIDTH) { // Clip right
      w = (WIDTH - x);
    }
    if(w > 0) { // Proceed only if width is positive
      uint8_t *pBuf = &buffer[(y / 8) * WIDTH + x],
               mask = 1 << (y & 7);
      switch(color) {
       case SSD1306_WHITE:               while(w--) { *pBuf++ |= mask; }; break;
       case SSD1306_BLACK: mask = ~mask; while(w--) { *pBuf++ &= mask; }; break;
       case SSD1306_INVERSE:             while(w--) { *pBuf++ ^= mask; }; break;
      }
    }
  }
}