#pragma once

#include "wled.h"

// http://bildr.org/2012/08/rotary-encoder-arduino/
//
// Inspired by the v1 usermods
// * rotary_encoder_change_effect
//
// v2 usermod that provides a rotary encoder-based UI.
//
// This usermod allows you to control:
//
// * LED Power (on/off) via button
// * Selected Effect or preset
//
// Dependencies
// * This usermod REQURES the ModeSortUsermod
//

#ifndef ENCODER_PIN_1
#define ENCODER_PIN_1 12
#endif

#ifndef ENCODER_PIN_2
#define ENCODER_PIN_2 14
#endif

#ifndef ENCODER_SW_PIN
#define ENCODER_SW_PIN 13
#endif

#ifndef ENCODER_TICKS_PER_DETENT
#define ENCODER_TICKS_PER_DETENT 4
#endif

#ifndef ENCODER_PRESET_START
#define ENCODER_PRESET_START 1
#endif

#ifndef ENCODER_PRESET_END
#define ENCODER_PRESET_END   10
#endif

static unsigned int encoder_state = 0;
static int encoder_value = 0;
static void IRAM_ATTR updateEncoder() {
    unsigned int MSb = digitalRead(ENCODER_PIN_1); // Read encoder pins
    unsigned int LSb = digitalRead(ENCODER_PIN_2);
    unsigned int encoded = (MSb << 1) | LSb;
    encoder_state = (encoder_state << 2) | encoded;

    switch (encoder_state & 0x0F) {
      case 0b1101:
      case 0b0100:
      case 0b0010:
      case 0b1011:
        // counter clockwise
        encoder_value--;
        break;
      
      case 0b1110:
      case 0b0111:
      case 0b0001:
      case 0b1000:
        // clockwise
        encoder_value++;
        break;

      default:
        break;
    }
  }

class RotaryControl : public Usermod {
private:
  unsigned long currentTime;
  unsigned long loopTime;
  unsigned char button_state = HIGH;
  unsigned char prev_button_state = HIGH;

  int last_encoder_value;
  int currentPreset = 0;

public:
  /*
   * setup() is called once at boot. WiFi is not yet connected at this point.
   * You can use it to initialize variables, sensors or similar.
   */
  void setup() {
    pinMode(ENCODER_PIN_1, INPUT_PULLUP);
    pinMode(ENCODER_PIN_2, INPUT_PULLUP);
    pinMode(ENCODER_SW_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_1), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_2), updateEncoder, CHANGE);

    currentTime = millis();
    loopTime = currentTime;
  }

  /*
   * connected() is called every time the WiFi is (re)connected
   * Use it to initialize network interfaces
   */
  void connected() {
  }

  /*
   * loop() is called continuously. Here you can check for events, read sensors,
   * etc.
   *
   * Tips:
   * 1. You can use "if (WLED_CONNECTED)" to check for a successful network
   * connection. Additionally, "if (WLED_MQTT_CONNECTED)" is available to check
   * for a connection to an MQTT broker.
   *
   * 2. Try to avoid using the delay() function. NEVER use delays longer than 10
   * milliseconds. Instead, use a timer check as shown here.
   */
    void loop() {
        currentTime = millis(); // get the current elapsed time

        if (currentTime >= (loopTime + 2)) {
            // 2ms since last check of encoder = 500Hz
            //button_state = digitalRead(ENCODER_SW_PIN);
            // if (prev_button_state != button_state) {
            //   // change power
            //   Serial.println("Power!");
            // }

            int encoder_capture = encoder_value;
            int encoder_change = encoder_capture - last_encoder_value;
            int preset = currentPreset;
            while (abs(encoder_change) >= ENCODER_TICKS_PER_DETENT) {
                if (encoder_change > 0) {
                    // Serial.println("Preset Up"); Serial.println(encoder_capture);

                    encoder_change -= ENCODER_TICKS_PER_DETENT;
                    if (++preset > ENCODER_PRESET_END) {
                        preset = ENCODER_PRESET_START;
                    }
                }

                if (encoder_change < 0) {
                    // Serial.println("Preset Down"); Serial.println(encoder_capture);
                    
                    encoder_change += ENCODER_TICKS_PER_DETENT;
                    if (--preset < ENCODER_PRESET_START) {
                        preset = ENCODER_PRESET_END;
                    }
                }

                last_encoder_value = encoder_capture;
            }

            if (preset != currentPreset) {
                currentPreset = preset;
                applyPreset(currentPreset);
                // Serial.print("PresetIndex="); Serial.println(currentPreset);
            }

            loopTime = currentTime; // Updates loopTime
        }
    }

  /*
   * addToJsonInfo() can be used to add custom entries to the /json/info part of
   * the JSON API. Creating an "u" object allows you to add custom key/value
   * pairs to the Info section of the WLED web UI. Below it is shown how this
   * could be used for e.g. a light sensor
   */
  /*
  void addToJsonInfo(JsonObject& root)
  {
    int reading = 20;
    //this code adds "u":{"Light":[20," lux"]} to the info object
    JsonObject user = root["u"];
    if (user.isNull()) user = root.createNestedObject("u");
    JsonArray lightArr = user.createNestedArray("Light"); //name
    lightArr.add(reading); //value
    lightArr.add(" lux"); //unit
  }
  */

  /*
   * addToJsonState() can be used to add custom entries to the /json/state part
   * of the JSON API (state object). Values in the state object may be modified
   * by connected clients
   */
  void addToJsonState(JsonObject &root) {
    // root["user0"] = userVar0;
  }

  /*
   * readFromJsonState() can be used to receive data clients send to the
   * /json/state part of the JSON API (state object). Values in the state object
   * may be modified by connected clients
   */
  void readFromJsonState(JsonObject &root) {
    userVar0 = root["user0"] |
               userVar0; // if "user0" key exists in JSON, update, else keep old
                         // value if (root["bri"] == 255)
                         // Serial.println(F("Don't burn down your garage!"));
  }

  /*
   * getId() allows you to optionally give your V2 usermod an unique ID (please
   * define it in const.h!). This could be used in the future for the system to
   * determine whether your usermod is installed.
   */
  uint16_t getId() {
    return USERMOD_ID_ROTARY_CONTROL;
  }
};
