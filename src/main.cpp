#include <Arduino.h>
#include "buffer.h"

void setup() {
    Serial.begin(115200);
    Serial.dtr(false);
    buffer_init();

}

void loop() {
    buffer_loop();
}

