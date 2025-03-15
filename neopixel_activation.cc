#include <Adafruit_NeoPixel.h>

#define LED_PIN1 5   // D1
#define LED_PIN2 6   // D2
#define LED_PIN3 7   // D3
#define NUM_PIXELS 1 // Single NeoPixel per pin
#define NUM_TOTAL 3  // Number of rim points
#define DEFAULT_COLOR Adafruit_NeoPixel::Color(150, 225, 225)

Adafruit_NeoPixel pixel1(NUM_PIXELS, LED_PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixel2(NUM_PIXELS, LED_PIN2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixel3(NUM_PIXELS, LED_PIN3, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel *pixels[] = {&pixel1, &pixel2, &pixel3};
bool ledStates[NUM_TOTAL] = {false, false, false};
int currentActive = -1; // Track currently active LED

void setup()
{
    Serial.begin(9600);
    for (int i = 0; i < NUM_TOTAL; i++)
    {
        pixels[i]->begin();
        pixels[i]->setBrightness(40);
        pixels[i]->show();
    }
}

void updateLED(int index, bool state)
{
    if (index < 0 || index >= NUM_TOTAL)
        return;

    if (ledStates[index] != state)
    {
        pixels[index]->setPixelColor(0, state ? DEFAULT_COLOR : 0);
        pixels[index]->show();
        ledStates[index] = state;
    }
}

void processCommand(char cmd)
{
    switch (cmd)
    {
    case '0':
    case '1':
    case '2':
    {
        int newActive = cmd - '0';
        if (newActive != currentActive)
        {
            // Turn off previous active LED
            if (currentActive != -1)
            {
                updateLED(currentActive, false);
            }
            // Turn on new active LED
            updateLED(newActive, true);
            currentActive = newActive;
        }
        break;
    }
    case 'x':
    {
        if (currentActive != -1)
        {
            updateLED(currentActive, false);
            currentActive = -1;
        }
        break;
    }
    }
}

void loop()
{
    if (Serial.available() > 0)
    {
        char input = Serial.read();
        processCommand(input);
    }
    delay(50);
}
