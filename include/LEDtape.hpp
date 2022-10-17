#pragma once
#include <Arduino.h>
#include <cstdlib> //abs,rand
#include "APA102.h"

class LEDtape
{
    public:
        enum class Color{
            RED,
            GREEN,
            BLUE,
        };
        enum class Eye{
            RIGHTEYE,
            LEFTEYE,
            CENTEREYE,
        };

        //コンストラクター
        LEDtape(const uint8_t datapin, const uint8_t clockpin, const uint16_t led_number, const uint8_t brightness);
        //シンプルに点滅を繰り返す
        lit(Color color, double rate_time);
        //目
        eye(Eye eye, double rate_time);
        //虹色に光る
        rainbow();
        //彩度(s)を変えてsin波のように
        stulationsin(uint16_t h, double rate);
        //ガンダム
        gundam();

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
    }else if(color == Color::BLUE){
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

    //write led
    ledStrip.write(leds_color, LED_NUMBER, brightness);
}

LEDtape::eye(LEDtape::Eye eye, double rate_time)
{
    //color
    rgb_color leds_color[LED_NUMBER];
    uint16_t LED_CENTER = LED_NUMBER / 2;
    uint8_t EYE_SIZE = 10;
    uint8_t EYE_MOVEMENT = 10;

    if(eye == Eye::RIGHTEYE){
        //right eye
        for(int i = 0; i < LED_NUMBER; ++i){
            if(EYE_SIZE == abs(i - LED_CENTER + EYE_MOVEMENT)){
                leds_color[i] = rgb_color(255, 0, 0); //一旦RED
            }else{
                leds_color[i] = rgb_color(0,0,0); //目以外は0,0,0
            }
        }
    }else if(eye == Eye::LEFTEYE){
        //left eye
        for(int i = 0; i < LED_NUMBER; ++i){
            if(EYE_SIZE > abs(i - LED_CENTER - EYE_MOVEMENT)){
                leds_color[i] = rgb_color(255, 0, 0); //一旦RED
            }else{
                leds_color[i] = rgb_color(0,0,0); //目以外は0,0,0
            }
        }
    }else{
        //center eye
        for(int i = 0; i < LED_NUMBER; ++i){
            if(EYE_SIZE == abs(i - LED_CENTER)){
                leds_color[i] = rgb_color(255, 0, 0); //一旦RED
            }else{
                leds_color[i] = rgb_color(0,0,0); //目以外は0,0,0
            }
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

    //write led
    ledStrip.write(leds_color, LED_NUMBER, brightness);
}

LEDtape::rainbow()
{
    rgb_color leds_color[LED_NUMBER];
    //rainbow
    for(int i = 0; i < LED_NUMBER; ++i){
        uint8_t p = (millis() >> 4) - i * 8;
        leds_color[i] = hsvToRgb((uint32_t)p * 359 / 256, 255, 255);
    }

    //write led
    ledStrip.write(leds_color, LED_NUMBER, BRIGHTNESS);
}

LEDtape::stulationsin(uint16_t h, double rate)
{
    rgb_color leds_color[LED_NUMBER];
    //Satulation sin

    uint8_t p = abs(255*sin(millis()*rate));
    for(int i = 0; i < LED_NUMBER; ++i){
        leds_color[i] = hsvToRgb(h, p, 255);
    }

    //write led
    ledStrip.write(leds_color, LED_NUMBER, BRIGHTNESS);
}

LEDtape::gundam()
{
    rgb_color leds_color[LED_NUMBER];
    srandom(millis());
    int rate_time = 30;

    static bool isLightOn = false;

    if(millis() - pretime > rate_time && isLightOn == false){
        //gundam
        for(int i = 0; i < LED_NUMBER; ++i){
            uint8_t r = rand() % (4); //零剰余の回避
            if(1 == r ){
                leds_color[i] = rgb_color(255, 0, 0);
            }else{
                leds_color[i] = rgb_color(1, 0, 0);
            }
            //write led
            ledStrip.write(leds_color, LED_NUMBER, BRIGHTNESS);    
        }
        isLightOn = true;

    }
    //timer reset
    if(millis() - pretime > rate_time * 2){
        pretime = millis();
        isLightOn = false;
    }
}


rgb_color LEDtape::hsvToRgb(uint16_t h, uint8_t s, uint8_t v)
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