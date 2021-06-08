

// Demo code for artifical horizon display
// Written by Bodmer for a 160 x 128 TFT display
// 15/8/16

#include <SPI.h>
#include <Arduino.h>
#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_I2CDevice.h>

// Use ONE of these three highly optimised libraries, comment out other two!

// For S6D02A1 based TFT displays
// #include <TFT_S6D02A1.h>         // Bodmer's graphics and font library for S6D02A1 driver chip
// TFT_S6D02A1 tft = TFT_S6D02A1(); // Invoke library, pins defined in User_Setup.h
//                                    https://github.com/Bodmer/TFT_S6D02A1

// For ST7735 based TFT displays
#include <TFT_ST7735.h>        // Bodmer's graphics and font library for ST7735 driver chip
TFT_ST7735 tft = TFT_ST7735(); // Invoke library, pins defined in User_Setup.h
// //                                    https://github.com/Bodmer/TFT_ST7735

// #include <Adafruit_ST7735.h>
// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// For ILI9341 based TFT displays (note sketch is currently setup for a 160 x 128 display)
//#include <TFT_ILI9341.h>         // Bodmer's graphics and font library for ILI9341 driver chip
//TFT_ILI9341 tft = TFT_ILI9341(); // Invoke library, pins defined in User_Setup.h
//                                    https://github.com/Bodmer/TFT_ILI9341

#define REDRAW_DELAY 16 // minimum delay in milliseconds between display updates

#define HOR 205 // Horizon circle outside radius (205 is corner to corner

#define BROWN 0x5140    //0x5960
#define SKY_BLUE 0x02B5 //0x0318 //0x039B //0x34BF
#define DARK_RED 0x8000
#define DARK_GREY 0x39C7

#define XC 80 // x coord of centre of horizon
#define YC 64 // y coord of centre of horizon

#define ANGLE_INC 1 // Angle increment for arc segments, 1 will give finer resolution, 2 or more gives faster rotation

#define DEG2RAD 0.0174532925

int roll_angle = 180; // These must be initialed to 180 so updateHorizon(0); in setup() draws

int last_roll = 0; // the whole horizon graphic
int last_pitch = 0;

int roll_delta = 90; // This is used to set arc drawing direction, must be set to 90 here

// Variables for test only
int test_angle = 0;
int delta = ANGLE_INC;

unsigned long redrawTime = 0;

void drawHorizon(int roll, int pitch)
{
  // Calculate coordinates for line start
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);

  int16_t x0 = sx * HOR;
  int16_t y0 = sy * HOR;
  int16_t xd = 0;
  int16_t yd = 1;
  int16_t xdn = 0;
  int16_t ydn = 0;

  if (roll > 45 && roll < 135)
  {
    xd = -1;
    yd = 0;
  }
  if (roll >= 135)
  {
    xd = 0;
    yd = -1;
  }
  if (roll < -45 && roll > -135)
  {
    xd = 1;
    yd = 0;
  }
  if (roll <= -135)
  {
    xd = 0;
    yd = -1;
  }

  if ((roll != last_roll) && ((abs(roll) > 35) || (pitch != last_pitch)))
  {
    xdn = 4 * xd;
    ydn = 4 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);
    xdn = 3 * xd;
    ydn = 3 * yd;
    tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
    tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);
  }
  xdn = 2 * xd;
  ydn = 2 * yd;
  tft.drawLine(XC - x0 - xdn, YC - y0 - ydn - pitch, XC + x0 - xdn, YC + y0 - ydn - pitch, SKY_BLUE);
  tft.drawLine(XC - x0 + xdn, YC - y0 + ydn - pitch, XC + x0 + xdn, YC + y0 + ydn - pitch, BROWN);

  tft.drawLine(XC - x0 - xd, YC - y0 - yd - pitch, XC + x0 - xd, YC + y0 - yd - pitch, SKY_BLUE);
  tft.drawLine(XC - x0 + xd, YC - y0 + yd - pitch, XC + x0 + xd, YC + y0 + yd - pitch, BROWN);

  tft.drawLine(XC - x0, YC - y0 - pitch, XC + x0, YC + y0 - pitch, TFT_WHITE);

  last_roll = roll;
  last_pitch = pitch;
}

// #########################################################################
// Draw the information
// #########################################################################

void drawInfo(void)
{
  // Update things near middle of screen first (most likely to get obscured)

  // Level wings graphic
  tft.fillRect(80 - 1, 64 - 1, 3, 3, TFT_RED);
  tft.drawFastHLine(80 - 30, 64, 24, TFT_RED);
  tft.drawFastHLine(80 + 30 - 24, 64, 24, TFT_RED);
  tft.drawFastVLine(80 - 30 + 24, 64, 3, TFT_RED);
  tft.drawFastVLine(80 + 30 - 24, 64, 3, TFT_RED);

  tft.drawFastHLine(80 - 12, 64 - 40, 24, TFT_WHITE);
  tft.drawFastHLine(80 - 6, 64 - 30, 12, TFT_WHITE);
  tft.drawFastHLine(80 - 12, 64 - 20, 24, TFT_WHITE);
  tft.drawFastHLine(80 - 6, 64 - 10, 12, TFT_WHITE);

  tft.drawFastHLine(80 - 6, 64 + 10, 12, TFT_WHITE);
  tft.drawFastHLine(80 - 12, 64 + 20, 24, TFT_WHITE);
  tft.drawFastHLine(80 - 6, 64 + 30, 12, TFT_WHITE);
  tft.drawFastHLine(80 - 12, 64 + 40, 24, TFT_WHITE);

  tft.setTextColor(TFT_WHITE);
  tft.setCursor(80 - 12 - 13, 64 - 20 - 3);
  tft.print("10");
  tft.setCursor(80 + 12 + 1, 64 - 20 - 3);
  tft.print("10");
  tft.setCursor(80 - 12 - 13, 64 + 20 - 3);
  tft.print("10");
  tft.setCursor(80 + 12 + 1, 64 + 20 - 3);
  tft.print("10");

  tft.setCursor(80 - 12 - 13, 64 - 40 - 3);
  tft.print("20");
  tft.setCursor(80 + 12 + 1, 64 - 40 - 3);
  tft.print("20");
  tft.setCursor(80 - 12 - 13, 64 + 40 - 3);
  tft.print("20");
  tft.setCursor(80 + 12 + 1, 64 + 40 - 3);
  tft.print("20");

  // Display justified angle value near bottom of screen
  tft.setTextColor(TFT_YELLOW, BROWN); // Text with background
  tft.setTextDatum(MC_DATUM);          // Centre middle justified
  tft.setTextPadding(24);              // Padding width to wipe previous number
  tft.drawNumber(last_roll, 80, 120, 1);
}

// #########################################################################
// Update the horizon with a new angle (angle in range -180 to +180)
// #########################################################################
void updateHorizon(int angle, int pitch)
{
  bool draw = 1;
  int delta_pitch = 0;
  int pitch_error = 0;
  int delta_roll = 0;
  while ((last_pitch != pitch) || (last_roll != angle))
  {
    delta_pitch = 0;
    delta_roll = 0;

    if (last_pitch < pitch)
    {
      delta_pitch = 1;
      pitch_error = pitch - last_pitch;
    }

    if (last_pitch > pitch)
    {
      delta_pitch = -1;
      pitch_error = last_pitch - pitch;
    }

    if (last_roll < angle)
      delta_roll = 1;
    if (last_roll > angle)
      delta_roll = -1;

    if (delta_roll == 0)
    {
      if (pitch_error > 1)
        delta_pitch *= 2;
    }

    drawHorizon(last_roll + delta_roll, last_pitch + delta_pitch);
    drawInfo();
  }
}

// #########################################################################
// Function to generate roll angles for testing only
// #########################################################################

int angleGenerator(int maxAngle)
{

  // Synthesize a smooth +/- 50 degree roll value for testing
  delta++;
  if (delta >= 360)
    test_angle = 0;
  test_angle = (maxAngle + 1) * sin((delta)*DEG2RAD);

  // Clip value so we hold angle near peak
  if (test_angle > maxAngle)
    test_angle = maxAngle;
  if (test_angle < -maxAngle)
    test_angle = -maxAngle;

  return test_angle;
}

void testRoll(void)
{
  // tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  // tft.setTextDatum(TC_DATUM); // Centre middle justified
  // tft.drawString("Roll test", 64, 10, 1);

  for (int a = 0; a < 360; a++)
  {
    //delay(REDRAW_DELAY / 2);
    updateHorizon(angleGenerator(180), 0);
  }
  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM); // Centre middle justified
  tft.drawString("         ", 64, 10, 1);
}

void testPitch(void)
{

  // tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  // tft.setTextDatum(TC_DATUM); // Centre middle justified
  // tft.drawString("Pitch test", 64, 10, 1);

  for (int p = 0; p > -80; p--)
  {
    delay(REDRAW_DELAY / 2);
    updateHorizon(0, p);
  }

  for (int p = -80; p < 80; p++)
  {
    delay(REDRAW_DELAY / 2);
    updateHorizon(0, p);
  }

  for (int p = 80; p > 0; p--)
  {
    delay(REDRAW_DELAY / 2);
    updateHorizon(0, p);
  }

  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM); // Centre middle justified
  tft.drawString("          ", 64, 10, 1);
}

// #########################################################################
// Setup, runs once on boot up
// #########################################################################

void setup(void)
{
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(1);

  tft.fillRect(0, 0, 160, 64, SKY_BLUE);
  tft.fillRect(0, 64, 160, 64, BROWN);
  delay(1000);

  // Draw the horizon graphic
  drawHorizon(0, 0);
  delay(1000);
  drawInfo();
  delay(1000); // Wait to permit visual check

  // Test roll and pitch
  testRoll();
  delay(2000);
  testPitch();

  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM); // Centre middle justified
  tft.drawString("Random", 64, 10, 1);
}

// #########################################################################
// Main loop, keeps looping around
// #########################################################################

void loop()
{

  // Refresh the display at regular intervals
  if (millis() > redrawTime)
  {
    redrawTime = millis() + REDRAW_DELAY;

    // Roll is in degrees in range +/-180
    int roll = random(361) - 180;

    // Pitch is in y coord (pixel) steps, 20 steps = 10 degrees on drawn scale
    // Maximum pitch shouls be in range +/- 80 with HOR = 172
    int pitch = random(161) - 80;

    updateHorizon(roll, pitch);
  }
}
