/**
 * Based on: 
 * - https://github.com/wilhelmzeuschner/arduino_thermal_camera_with_sd_and_img_processing.
 * - https://github.com/sparkfun/SparkFun_MLX90640_Arduino_Example
 * 
 * Hardware:
 * - ESP32: https://www.espressif.com/en/products/hardware/esp32-devkitc/overview
 * - Sensor: https://shop.pimoroni.com/products/mlx90640-thermal-camera-breakout
 * - Display: https://www.amazon.de/gp/product/B07DPMV34R/, https://www.pjrc.com/store/display_ili9341.html
 */


#include <TFT_eSPI.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define EMMISIVITY 0.95
#define INTERPOLATE false

#define C_BLUE Display.color565(0,0,255)
#define C_RED Display.color565(255,0,0)
#define C_GREEN Display.color565(0,255,0)
#define C_WHITE Display.color565(255,255,255)
#define C_BLACK Display.color565(0,0,0)
#define C_LTGREY Display.color565(200,200,200)
#define C_DKGREY Display.color565(80,80,80)
#define C_GREY Display.color565(127,127,127)

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
#define TA_SHIFT 8 //Default shift for MLX90640 in open air
paramsMLX90640 mlx90640;


TFT_eSPI Display = TFT_eSPI();


// Added for measure Temp
boolean measure = true;
float centerTemp;
unsigned long tempTimeNow = 0;
unsigned long tempTimeFPS = 0;
unsigned long tempTimeRead = 0;
unsigned long tempTimeProcess = 0;
unsigned long tempTimeDraw = 0;

// start with some initial colors
float minTemp = 20.0;
float maxTemp = 40.0;


// variables for interpolated colors
byte red, green, blue;

// variables for row/column interpolation
float intPoint, val, a, b, c, d, ii;
int x, y, i, j;

// raw sensor data
static uint16_t mlx90640Frame[834];

// array for the 32 x 24 measured tempValues
static float tempValues[32*24];

// Output size
#define O_WIDTH 224
#define O_HEIGHT 168
#define O_RATIO O_WIDTH/32

float **interpolated = NULL;
uint16_t *imageData = NULL;

void setup() {
  Serial.begin(115200);
  Serial.println("Hello.");

  // Connect thermal sensor.
  Wire.begin();
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0) {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring.");
    for(;;){}
  }
  else {
    Serial.println("MLX90640 online!");
  }
  // Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0) Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) Serial.println("Parameter extraction failed");
  // Set refresh rate
  // 0x00 -> 0.25Hz
  // 0x01 -> 0.5Hz
  // 0x02 -> 1Hz
  // 0x03 -> 2Hz
  // 0x04 -> 4Hz
  // 0x05 -> 8Hz
  // 0x06 -> 16Hz
  // 0x07 -> 32Hz
  MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 4Hz effective - Works best on esp8266

  // Once EEPROM has been read at 400kHz we can increase
  Wire.setClock(800000); // esp8266 supports up to 800kHz

  // Set up Display.
  Serial.println("Setting up display...");
  pinMode(TFT_DC, OUTPUT);
  SPI.begin();
  SPI.setFrequency(80000000L);
  Display.begin();
  //Display.setRotation(3);
  Display.fillScreen(C_BLACK);


  // Prepare interpolated array
  interpolated = (float **)malloc(O_HEIGHT * sizeof(float *));
  for (int i=0; i<O_HEIGHT; i++) {
    interpolated[i] = (float *)malloc(O_WIDTH * sizeof(float));
  }

  // Prepare imageData array
  imageData = (uint16_t *)malloc(O_WIDTH * O_HEIGHT * sizeof(uint16_t));

  // draw the legend rainbow once
  drawRainbow();
}

int frames = 0;
void loop() {
  tempTimeNow = millis();
  if(tempTimeNow - tempTimeFPS > 10000) {
    Serial.print((frames*1000.0)/(tempTimeNow - tempTimeFPS));
    Serial.print(" fps - ");
    Serial.print(tempTimeRead/frames);
    Serial.print("ms reading, ");
    Serial.print(tempTimeProcess/frames);
    Serial.print("ms processing, ");
    Serial.print(tempTimeDraw/frames);
    Serial.println("ms drawing");
    tempTimeFPS = tempTimeNow;
    
    frames = 0;
    tempTimeRead = 0;
    tempTimeProcess = 0;
    tempTimeDraw = 0;
    tempTimeNow = millis();
  }
  
  readFrame();
  tempTimeRead += millis() - tempTimeNow;
  tempTimeNow = millis();
  
  computeTemps();
  tempTimeProcess += millis() - tempTimeNow;
  tempTimeNow = millis();
  
  setTempScale();
  drawPicture();
  if(frames%2 == 1) {
    drawMeasurement();
    drawLegend();
  }
  tempTimeDraw += millis() - tempTimeNow;

  frames++; 
}

// Read pixel data from MLX90640.
void readFrame() {
  int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
  if (status < 0)
  {
    Serial.print("GetFrame Error: ");
    Serial.println(status);
  }
}

void computeTemps() {
  float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
  float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature

  MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMMISIVITY, tr, tempValues);
}

// https://stackoverflow.com/questions/28495390/thermal-imaging-palette
const uint16_t camColors[] = {
  0x0001,0x0001,0x0002,0x0003,0x0003,0x0004,0x0005,0x0005,0x0006,0x0007,0x0007,
  0x0008,0x0009,0x0009,0x000A,0x000B,0x000B,0x000C,0x000D,0x000D,0x080D,0x080E,
  0x080E,0x100E,0x100F,0x100F,0x180F,0x1810,0x1810,0x2010,0x2810,0x2810,0x2811,
  0x3011,0x3011,0x3811,0x4012,0x4012,0x4812,0x5012,0x5012,0x5812,0x6012,0x6012,
  0x6812,0x6813,0x6813,0x7013,0x7813,0x7813,0x8013,0x8813,0x8813,0x8812,0x9012,
  0x9012,0x9812,0xA012,0xA012,0xA812,0xB012,0xB012,0xB011,0xB031,0xB031,0xB831,
  0xB851,0xB851,0xC070,0xC090,0xC090,0xC0AF,0xC8AF,0xC8AF,0xC8CE,0xC8EE,0xC8EE,
  0xC8ED,0xC90D,0xC90D,0xC90C,0xD12C,0xD12C,0xD12B,0xD14B,0xD14B,0xD16A,0xD169,
  0xD169,0xD189,0xD188,0xD188,0xD988,0xD9A7,0xD9A7,0xD9A6,0xD9C6,0xD9C6,0xD9C5,
  0xD9E4,0xD9E4,0xD9E3,0xDA03,0xDA03,0xDA22,0xE222,0xE222,0xE242,0xE241,0xE241,
  0xE261,0xE281,0xE281,0xE2A0,0xE2C0,0xE2C0,0xE2E0,0xEAE0,0xEAE0,0xEB00,0xEB00,
  0xEB20,0xEB20,0xEB40,0xEB40,0xEB60,0xEB60,0xEB80,0xEB80,0xEBA0,0xEBA0,0xEBC0,
  0xEBC0,0xEBE0,0xEBE0,0xEC00,0xEC00,0xF400,0xF400,0xF420,0xF440,0xF440,0xF460,
  0xF480,0xF480,0xF4A0,0xF4C0,0xF4C0,0xF4E0,0xF500,0xF500,0xF520,0xF540,0xF540,
  0xF560,0xF580,0xF580,0xF5A0,0xF5C0,0xF5C0,0xF5E0,0xF600,0xF600,0xF620,0xF640,
  0xF640,0xF660,0xF680,0xF680,0xF681,0xF6A1,0xF6A1,0xFEA1,0xFEC1,0xFEC1,0xFEC2,
  0xFEC3,0xFEC3,0xFEE3,0xFEE4,0xFEE4,0xFF04,0xFF05,0xFF05,0xFF06,0xFF27,0xFF27,
  0xFF28,0xFF48,0xFF48,0xFF49,0xFF4A,0xFF4A,0xFF4B,0xFF6C,0xFF6C,0xFF6D,0xFF6E,
  0xFF6E,0xFF6F,0xFF70,0xFF70,0xFF91,0xFF92,0xFF92,0xFF93,0xFF94,0xFF94,0xFF95,
  0xFFB5,0xFFB5,0xFFB6,0xFFB7,0xFFB7,0xFFB8,0xFFB9,0xFFB9,0xFFDA,0xFFDB,0xFFDB,
  0xFFDC,0xFFDD,0xFFDD,0xFFFD,
};

void drawPicture() {
  float degreesPerColor = 223.0 / (maxTemp - minTemp);
  for (y=0; y<24; y++) {
    for (x=0; x<32; x++) {
      float t = tempValues[(31-x) + (y*32)];
      // uint8_t colorIndex = map(t, minTemp, maxTemp, 0, sizeof(camColors)/sizeof(uint16_t) - 1);
      uint8_t colorIndex = (t - minTemp) * degreesPerColor;
      Display.fillRect(8 + x*7, 8 + y*7, 7, 7, camColors[colorIndex]);
    }
  }
}

void setTempScale() {
  minTemp = 1000;
  maxTemp = -1000;

  for (i = 0; i < 768; i++) {
    minTemp = min(minTemp, tempValues[i]);
    maxTemp = max(maxTemp, tempValues[i]);
  }
}

void drawRainbow() {
  for (i = 0; i < 224; i += 1) {
    Display.drawFastVLine(8+ + i, 292, 20, camColors[i]);
  }
}

// Draw a legend.
void drawLegend() {
  Display.setTextFont(2);
  Display.setTextSize(1);
  Display.setCursor(8, 272);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.print(String(minTemp).substring(0, 5));

  Display.setCursor(192, 272);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.print(String(maxTemp).substring(0, 5));

  Display.setTextFont(NULL);
}


// Draw a circle + measured value.
void drawMeasurement() {

  // Mark center measurement
  Display.drawCircle(120, 8+84, 3, TFT_WHITE);

  // Measure and print center temperature
  centerTemp = (tempValues[383 - 16] + tempValues[383 - 15] + tempValues[384 + 15] + tempValues[384 + 16]) / 4;
  Display.setCursor(86, 214);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.setTextFont(2);
  Display.setTextSize(2);
  Display.print(String(centerTemp).substring(0, 5) + " °C");
}
