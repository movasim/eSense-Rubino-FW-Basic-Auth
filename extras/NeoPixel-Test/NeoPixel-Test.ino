#include <Adafruit_NeoPixel.h>

#define NeoPixel 14

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, NeoPixel, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  pixels.begin(); 
}

void loop() {
  RGB_color(255, 255, 255, 128);
  Serial.println("WHITE");
  delay (5000);
  
  RGB_color(0, 0, 255, 128); // Callibrating | 
  Serial.println("BLUE");
  delay (5000);

  RGB_color(0, 255, 0, 128); // Excellent (IAQ = 0-50) | GREEN
  Serial.println("GREEN");
  delay (5000);
  
  RGB_color(50, 255, 50, 128); // Good (IAQ = 51-100) | 
  Serial.println("LIGHT-GREEN");
  delay (5000);
  
  RGB_color(255, 170, 0, 128); // Lightly Polluted (IAQ = 101-150) | YELLOW
  Serial.println("YELLOW");
  delay (5000);
  
  RGB_color(255, 34, 0, 128); // Moderately Polluted (IAQ = 151-200) | 
  Serial.println("ORANGE");
  delay (5000);
  
  RGB_color(255, 0, 0, 128); // Heavily Polluted (IAQ = 201-250) | 
  Serial.println("RED");
  delay (5000);
  
  RGB_color(255, 0, 51, 128); // Severely Polluted (IAQ = 251-350) | 
  Serial.println("MAGENTA");
  delay (5000);
  
  RGB_color(255, 51, 119, 128); // Extremely Polluted (IAQ > 351) | 
  Serial.println("PINK");
  delay (5000);
}

void RGB_color(int red_light_value, int green_light_value, int blue_light_value, uint16_t brightness)
 {
  pixels.setPixelColor(0, pixels.Color((brightness*red_light_value/255), (brightness*green_light_value/255), (brightness*blue_light_value/255)));
  pixels.show(); // This sends the updated pixel color to the hardware.
}
