#ifndef CONSTANTS_H
#define CONSTANTS_H

//measurement calibration ratios
#define PWM_LUX_RATIO 0.0256f //PWM lux ratio (to be updated)

//#define LUX_TARGET_LOW 2000
//#define LUX_TARGET_MEDIUM 5000
//#define LUX_TARGET_HIGH 15000

#define LUX_TARGET_LOW 10
#define LUX_TARGET_MEDIUM 11
#define LUX_TARGET_HIGH 12

//dev board definitions
//#define PUMP_PIN 7
//#define LIGHT_PIN 9
//#define WATER_LEVEL_PIN 10 // pin D7
//#define ADA_TIME_LIMIT 10000000ULL //10 seconds


//mcu board definitions
#define PUMP_PIN 15
#define LIGHT_PIN 16    
#define ADA_TIME_LIMIT 10000000ULL  //10 seconds
#define WATER_LEVEL_PIN 21
//#define WATER_LEVEL_PIN 4

//testing definitions
#define PUMP_COOLDOWN 15000000ULL //15 second cooldown

//real world definitions
//#define PUMP_COOLDOWN 300000000ULL //5 minutes



#endif