#pragma once
#include <Arduino.h>
#include "APA102.h"

class LEDtape
{
    public:
        enum class Color{
            RED,
            GREEN,
            BLUE,
        };

        //コンストラクター
        LEDtape(const uint8_t datapin, const uint8_t clockpin, const uint16_t led_number, const uint8_t brightness);
        //シンプルに点滅を繰り返す
        lit(Color color, double rate_time);
    private:
        uint8_t DATA_PIN;
        uint8_t CLOCK_PIN;
        uint16_t LED_NUMBER;
        uint8_t BRIGHTNESS; //0 ~ 31
        APA102 ledStrip;

        unsigned long pretime;

        const int FAINTNESS_RATE = 0.2;

        rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v);

};

LEDtape::LEDtape(const uint8_t datapin, const uint8_t clockpin, const uint16_t led_number, const uint8_t brightness)
: DATA_PIN(datapin), CLOCK_PIN(clockpin), LED_NUMBER(led_number), BRIGHTNESS(brightness), ledStrip(datapin, clockpin)
{
    pretime = millis();
}

LEDtape::lit(LEDtape::Color color, double rate_time)
{
    //color
    rgb_color leds_color[LED_NUMBER];
    if(color == Color::RED){
        //red
        for(int i = 0; i < LED_NUMBER; ++i){
            leds_color[i] = rgb_color(255, 0, 0);
        }
    }else if(color == Color::GREEN){
        //green
        for(int i = 0; i < LED_NUMBER; ++i){
            leds_color[i] = rgb_color(0, 255, 0);
        }
    }else{
        //blue
        for(int i = 0; i < LED_NUMBER; ++i){
            leds_color[i] = rgb_color(0, 0, 255);
        }
    }

    //brightness
    uint8_t brightness = BRIGHTNESS;
    if(millis() - pretime > rate_time){
        brightness = BRIGHTNESS * FAINTNESS_RATE;
    }
    //timer reset
    if(millis() - pretime > rate_time * 2){
        pretime = millis();
    }

    ledStrip.write(leds_color, LED_NUMBER, brightness);
}

rgb_color hsvToRgb(uint16_t h, uint8_t s, uint8_t v)
{
    uint8_t f = (h % 60) * 255 / 60;
    uint8_t p = (255 - s) * (uint16_t)v / 255;
    uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
    uint8_t r = 0, g = 0, b = 0;
    switch((h / 60) % 6){
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
    }
    return rgb_color(r, g, b);
}