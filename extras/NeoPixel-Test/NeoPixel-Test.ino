/*
  eSense Rubino.
  Environmental Sensing Device based on Espressif ESP8266, Bosch BME680
  and ROHM BH1750.

  Copyright (C) 2020  MOVASIM (https://movasim.com/)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  This program outputs in the Serial Monitor the Color that NeoPixel should display.

*/

#include <Adafruit_NeoPixel.h>

#define NeoPixel 14

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, NeoPixel, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  pixels.begin(); 
}

void loop() {
  Serial.println();
  Serial.println("== NeoPixel Test Started ==");
  RGB_color(255, 255, 255, 128);
  Serial.println("WHITE");
  delay (5000);
  
  RGB_color(0, 0, 255, 128);
  Serial.println("BLUE");
  delay (5000);

  RGB_color(0, 255, 0, 128);
  Serial.println("GREEN");
  delay (5000);
  
  RGB_color(50, 255, 50, 128);
  Serial.println("LIGHT-GREEN");
  delay (5000);
  
  RGB_color(255, 170, 0, 128);
  Serial.println("YELLOW");
  delay (5000);
  
  RGB_color(255, 34, 0, 128);
  Serial.println("ORANGE");
  delay (5000);
  
  RGB_color(255, 0, 0, 128); 
  Serial.println("RED");
  delay (5000);
  
  RGB_color(255, 0, 51, 128);
  Serial.println("MAGENTA");
  delay (5000);
  
  RGB_color(255, 51, 119, 128);
  Serial.println("PINK");
  Serial.println("== NeoPixel Test Finished ==");
  delay (5000);
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value, uint16_t brightness)
 {
  pixels.setPixelColor(0, pixels.Color((brightness*red_light_value/255), (brightness*green_light_value/255), (brightness*blue_light_value/255)));
  pixels.show(); // This sends the updated pixel color to the hardware.
}
