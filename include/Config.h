#ifndef CONFIG_
#define CONFIG_

#include "Pinout.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "TofSensors.h"

#define TOF_ADRESS_FRONT 0x30
#define TOF_ADRESS_LEFT 0x31
#define TOF_ADRESS_RIGHT 0x32

// Orden para los TOF y los encoders
#define LEFT 0
#define RIGHT 1
#define FRONT 2

// Valores para el tiempo en el que volver a evaluar el estado, 
// innececesario si aplicamos enconder
#define T_GIROD 1000
#define T_GIROI 1000
#define T_GIRO180 2000

//Para el display
//#define PRINT_POSITION

// Distancias y angulos de giro
#define GIRO90_D 90
#define GIRO90_I 90
#define GIRO180_D 180
#define DISTANCIA_BACK 40
#define DISTANCIA_FOR_2 100
#define DISTANCIA_FOR_1 200

// ----------------- Parametros robot -------

    // Dimensions (mm)
    const float ROBOT_WIDTH = 158.0;
    const float ROBOT_RADIUS = 45.0;

    // Motor ticks por grado
    const float TICKS_PER_DEGREE = 4.0;

// ----------------- Parametros PID -------

    // Parametros PID Pared
    #define kp 3.2
    #define ki 0.0
    #define kd 3.2
    #define D_OPT 100

    // Valores para la velocidad seguir pared
    #define V_MAX 200
    #define V_BASE 100

    #define vel_MAX 120
    #define vel_MIN 40

    
    // Parametros PID Encoders
    #define kp_EN 1
    #define ki_EN 0.0
    #define kd_EN 0.5

    //Punto de saturacion para el PID
    #ifndef VEL_MAX_ENC
        #define VEL_MAX_ENC 200.0
    #endif

// ----------------- Display -----------------

    #define SCREEN_WIDTH 128 
    #define SCREEN_HEIGHT 64 
    #define OLED_RESET     4 
    #define SCREEN_ADDRESS 0x3

// ----------------- PWM ---------------------

    #define PWM_FREC 5000 //frecuencia de la senal pwm
    #define PWM_RES 8// resolucion de la senal pwm. En nuestro caso 8 bits (0 - 255)
    #define PWM_CH_D 0 //canal para el pwm del motor derecho 
    #define PWM_CH_I 1 //canal para el pwm del motor izquierdo


#endif