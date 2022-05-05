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
#define T_GIROD 750
#define T_GIROI 525
#define T_GIRO180 2000

//Para el display
//#define PRINT_POSITION

// Distancias y angulos de giro
#define GIRO90_D 90
#define GIRO90_I 90
#define GIRO180_D 180
#define DISTANCIA_BACK 40
#define DISTANCIA_FOR 100

// ----------------- Parametros robot -------

    // Dimensions (mm)
    const float ROBOT_WIDTH = 83.0;
    const float ROBOT_RADIUS = 30.0;

    // Motor ticks por grado
    const float TICKS_PER_DEGREE = 4.0;

// ----------------- Parametros PID -------

    // Parametros PID Pared
    #define kp 0.5
    #define ki 0.0
    #define kd 0.0
    #define D_OPT 150

    // Valores para la velocidad seguir pared
    #define V_MAX 150
    #define V_BASE 50
    
    // Parametros PID Encoders
    #define kp_EN 0.5
    #define ki_EN 0.0
    #define kd_EN 0.0

    //Punto de saturacion para el PID
    #ifndef VEL_MAX_ENC
        #define VEL_MAX_ENC 255
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



#define NUM_MODOS_DISPLAY 6
enum modos_display
{
    DISPLAY_APAGADO,
    DISPLAY_MOSTRAR_TOF,
    DISPLAY_MOSTRAR_PARAMETROSPID,
    DISPLAY_MOSTRAR_LINEA,
    DISPLAY_MOSTRAR_ESTADO,
    DISPLAY_MOSTRAR_LOGO
};


#endif