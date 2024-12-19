#include "esp32-hal.h"
#include <MCP23017.h>
#define MCP23017_ADDR0 0x20
#define MCP23017_ADDR1 0x21
MCP23017 mcp0 = MCP23017(MCP23017_ADDR0);
MCP23017 mcp1 = MCP23017(MCP23017_ADDR1);

#define display_pin_count 30

class Display_static_LCD {
public:

  Display_static_LCD(uint8_t digit_count = 4, uint8_t segment_count = 7, uint8_t special_segment_count = 2, uint8_t esp_gpio_count = 13, uint8_t expander_gpio_count = 16, int* seperator_p = nullptr) {
    digits = digit_count;
    segments = segment_count;
    special_segments = special_segment_count;
    esp_gpios = esp_gpio_count;
    expander_gpios = expander_gpio_count;
    seperator = seperator_p;
  }

  void init(uint8_t pin_map[display_pin_count]) {

     for (int i = 0; i < (esp_gpios + expander_gpios); i++) {

      pin_array[i] = pin_map[i];

      //if (pin_array[i] < 50) {
      //  pinMode(pin_array[i], OUTPUT);
      //}
    }
    //pinMode(TX, OUTPUT);


    mcp0.init();
    mcp0.portMode(MCP23017Port::A, 0);                   //Port A as output
    mcp0.portMode(MCP23017Port::B, 0);                   //Port B as output
    mcp0.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A
    mcp0.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B

    mcp1.init();
    mcp1.portMode(MCP23017Port::A, 0);                   //Port A as output
    mcp1.portMode(MCP23017Port::B, 0);                   //Port B as output
    mcp1.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A
    mcp1.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B
  }

  void print(uint16_t value) {

    //12569
    //0x0000 1010 0000 0001

    uint8_t value0 = value >> 8;
    uint8_t value1 = value;
    uint16_t left_side = 0;
    uint16_t right_side = 0;

    if (value0 < 10) {
      left_side = (decimal_to_digit(0) << 9) + (decimal_to_digit(value0) << 2);

    } else {
      left_side = (decimal_to_digit(value0 / 16) << 9) + (decimal_to_digit(value0 % 16) << 2);
    }


    if (value1 < 10) {
      right_side = (decimal_to_digit(0) << 9) + (decimal_to_digit(value1) << 2);

    } else {
      right_side = (decimal_to_digit(value1 / 16) << 9) + (decimal_to_digit(value1 % 16) << 2);
    }

    wave_generator((left_side << 16) + (right_side << 2));
  }

  void print(uint16_t value, int duration) {
    timer = millis();
    while (millis() - timer < duration) {
      print(value);
    }
  }
  uint8_t decimal_to_digit(int value) {
    switch (value) {
      case 0:
        return 0b1111110;

        break;
      case 1:
        return 0b0110000;

        break;
      case 2:
        return 0b1101101;

        break;
      case 3:
        return 0b1111001;

        break;
      case 4:
        return 0b0110011;

        break;
      case 5:
        return 0b1011011;

        break;
      case 6:
        return 0b1011111;

        break;
      case 7:
        return 0b1110000;

        break;
      case 8:
        return 0b1111111;

        break;
      case 9:
        return 0b1111011;

        break;
      case 0xA:
        return 0b0000000;

        break;

      case 0xB:
        return 0b0110111;

        break;

      case 0xC:
        return 0b0011111;

        break;
    }
  }


  void wave_generator(uint32_t value = 0) {

    uint16_t mcp0d = 0b0;
    uint16_t mcp1d = 0b0;

    for (int i = 0; i < 28; i++) {

      if (value & (1 << (31 - i))) {
        if (pin_array[i] > 69) {
          //digitalWrite(pin_array[i], 1);
          mcp1d = mcp1d | (1 << (pin_array[i] - 70));

        } else {
          mcp0d = mcp0d | (1 << (pin_array[i] - 50));
        }
      } else {
        // if (pin_array[i] < 50) {
        //  digitalWrite(pin_array[i], 0);
      }
    }

    //mcpd = mcpd | ((*seperator) << 15);
    mcp0d = mcp0d | ((*seperator) << 8);  //COL
    mcp1d = mcp1d & ~(1 << 7);           //BP
    mcp0.write(mcp0d);
    mcp1.write(mcp1d);
    //Serial.printf("value:%x,  MCP0:%x,  MCP1:%x\n",value,mcp0d,mcp1d);
    //digitalWrite(pin_array[display_pin_count - 1], 0);
    delay(delta);


    /*for (int i = 0; i < 28; i++) {

        if (value & (1 << (31 - i))) {
          if (pin_array[i] < 50) {
            digitalWrite(pin_array[i], 0);
          }
        } else {
          if (pin_array[i] < 50) {
            digitalWrite(pin_array[i], 1);
          }
        }
      }*/
    mcp0d = ~mcp0d;
    mcp1d = ~mcp1d;
    mcp0.write(mcp0d);
    mcp1.write(mcp1d);
    //digitalWrite(pin_array[display_pin_count - 1], 1);
    delay(delta);
  }


private:
  uint8_t pins_com[5] = {};
  uint8_t pins_segment[16] = {};
  int digits = 4;
  int segments = 8;
  int special_segments = 1;
  int esp_gpios = 14;
  int expander_gpios = 16;
  int delta = 3;
  int* seperator = nullptr;

  unsigned long timer;

  uint8_t pin_array[display_pin_count];
};