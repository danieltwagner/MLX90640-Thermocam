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



const uint16_t camColors[] = {
  0x001F,0x003F,0x005F,0x009F,0x00BF,0x00DF,0x011F,0x013F,0x015F,0x019F,
  0x01BF,0x01DF,0x021F,0x023F,0x025F,0x029F,0x02BF,0x02DF,0x031F,0x033F,
  0x035F,0x039F,0x03BF,0x03DF,0x041F,0x043F,0x045F,0x049F,0x04BF,0x04DF,
  0x051F,0x053F,0x055F,0x059F,0x05BF,0x05DF,0x061F,0x063F,0x065F,0x069F,
  0x06BF,0x06FF,0x071F,0x073F,0x077F,0x079F,0x07BF,0x07FF,0x07FF,0x07FD,
  0x07FC,0x07FB,0x07F9,0x07F8,0x07F7,0x07F5,0x07F4,0x07F3,0x07F1,0x07F0,
  0x07EF,0x07ED,0x07EC,0x07EB,0x07E9,0x07E8,0x07E7,0x07E5,0x07E4,0x07E3,
  0x07E1,0x07E0,0x07E0,0x17E0,0x1FE0,0x27E0,0x37E0,0x3FE0,0x4FE0,0x57E0,
  0x5FE0,0x6FE0,0x77E0,0x7FE0,0x8FE0,0x97E0,0x9FE0,0xAFE0,0xB7E0,0xBFE0,
  0xCFE0,0xD7E0,0xDFE0,0xEFE0,0xF7E0,0xFFE0,0xFFE0,0xFFC0,0xFFA0,0xFF80,
  0xFF80,0xFF60,0xFF40,0xFF20,0xFF20,0xFF00,0xFEE0,0xFEE0,0xFEC0,0xFEA0,
  0xFE80,0xFE80,0xFE60,0xFE40,0xFE40,0xFE20,0xFE00,0xFDE0,0xFDE0,0xFDC0,
  0xFDA0,0xFD80,0xFD80,0xFD60,0xFD40,0xFD40,0xFD20,0xFD00,0xFCE0,0xFCE0,
  0xFCC0,0xFCA0,0xFCA0,0xFC80,0xFC60,0xFC40,0xFC40,0xFC20,0xFC00,0xFBE0,
  0xFBE0,0xFBC0,0xFBA0,0xFBA0,0xFB80,0xFB60,0xFB40,0xFB40,0xFB20,0xFB00,
  0xFB00,0xFAE0,0xFAC0,0xFAA0,0xFAA0,0xFA80,0xFA60,0xFA40,0xFA40,0xFA20,
  0xFA00,0xFA00,0xF9E0,0xF9C0,0xF9A0,0xF9A0,0xF980,0xF960,0xF960,0xF940,
  0xF920,0xF900,0xF900,0xF8E0,0xF8C0,0xF8A0,0xF8A0,0xF880,0xF860,0xF860,
  0xF840,0xF820,0xF800,0xF800,0xF800,0xF801,0xF802,0xF802,0xF803,0xF804,
  0xF804,0xF805,0xF806,0xF807,0xF807,0xF808,0xF809,0xF80A,0xF80A,0xF80B,
  0xF80C,0xF80D,0xF80D,0xF80E,0xF80F,0xF810,0xF810,0xF811,0xF812,0xF812,
  0xF813,0xF814,0xF815,0xF815,0xF816,0xF817,0xF818,0xF818,0xF819,0xF81A,
  0xF81B,0xF81B,0xF81C,0xF81D,
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
