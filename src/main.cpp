#include <Arduino.h>

//Cosas que hacer:
/*
f1 -> Arduino: Board config -> ESP32 Dev Module
f1 -> Arduino: Initialize (solo una vez)
f1 -> Arduino: Verify (compilar)
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PCF8575.h>
#include "TofSensors.h"
#include "Pinout.h"
#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "NavLaberinto.h"
#include "Logo.h"
#include "Display.h"

//------------------------- EL codigo  -------------------------------------------

// VARIABLES

// Maquina de estado para las fases del laberinto
typedef enum {RUN_MODE_DEFAULT, RUN_MODE_PID, RUN_MODE_GORIGHT, RUN_MODE_GOLEFT, RUN_MODE_GO180, RUN_MODE_GOBACK, RUN_MODE_STOP} runmode_t;
runmode_t run_mode = RUN_MODE_DEFAULT;

//Variables de la distancia de cada sensor
int distancias[3];
//int distanciaC;
int contador = 0; // solo para el inicio

//Variables globales de tiempo
bool enc_START=0;
bool enc_END=0;
int tiempo=0;
int tiempo_obj=0;
int t_backON=0;




 //Objeto global para el extensor de pines
PCF8575 pinExtensor =  PCF8575(0x20);

//objeto global para el display
//Ya no se usa
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); 

//Objeto que maneja los 3 sensores tof
TofSensors MySensors(pin_xshut_front, pin_xshut_right, pin_xshut_left, &pinExtensor); 

//Objetos para los dos motores (NO control de posicion)
Motor Motor_derecho(PIN_MOTOR_D_IN1, PIN_MOTOR_D_IN2, PIN_MOTOR_D_PWM, PWM_CH_D, PWM_FREC, PWM_RES); //Motor derecho
Motor Motor_izquierdo(PIN_MOTOR_I_IN1, PIN_MOTOR_I_IN2, PIN_MOTOR_I_PWM, PWM_CH_I, PWM_FREC, PWM_RES); //Motor izquierdo

// Objetos globales para los dos encoders
Encoder_p encoderL = Encoder_p(PIN_ENCODER_I_CA, PIN_ENCODER_I_CB , 4.0);
Encoder_p encoderR = Encoder_p(PIN_ENCODER_D_CA, PIN_ENCODER_D_CB , 4.0);

// Objeto que maneja los motores
NavLaberinto mismotores(&Motor_derecho, &Motor_izquierdo, &encoderL, &encoderR);



// ############ Callbacks interrupciones encoders ######

void header_encoderL()
{
  encoderL.actualizar_posicion();
}

void header_encoderR()
{
  encoderR.actualizar_posicion();
}



void setup(){

//    Wire.setClock(800000); //Igual esto es lo que fuerza la velocidad del reloj

    // Inicializamos el serial
    Serial.begin(9600);

    // Inicializamos el display
    display_config();

    // Configuracion de interrupciones
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_I_CA), header_encoderL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_D_CA), header_encoderR, CHANGE);


    // Se inicializan los pines de los motores
    Motor_derecho.init();
    Motor_izquierdo.init();
    // Se inicializan los pines de los tof
    MySensors.init();
    // Se inicializa el extensor de pines
    pinExtensor.begin();
    // Se cambia la direccion de memoria de los sensores tof y se bootean
    MySensors.setID(TOF_ADRESS_FRONT, TOF_ADRESS_RIGHT, TOF_ADRESS_LEFT);
    
    //Logito y cosas
    display_printLogo();
    delay(1500);
    display.clearDisplay();

}
    
// No usado
void loop(void)
{
  /*
  // solo para entrar en el laberinto
  if(contador == 0){
    misMotores.avanzar();
    //delay(2000); // que alguien cambie esta basura
  }
  */
  // leo sensores
  //if(MySensors.getDistance(FRONT)!=-1) distancias[FRONT] = MySensors.getDistance(FRONT);
  //if(MySensors.getDistance(RIGHT)!=-1) distancias[RIGHT] = MySensors.getDistance(RIGHT);
  //if(MySensors.getDistance(LEFT)!=-1) distancias[LEFT] = MySensors.getDistance(LEFT);

  distancias[FRONT] = MySensors.getDistance(FRONT);
  distancias[RIGHT] = MySensors.getDistance(RIGHT);
  distancias[LEFT] = MySensors.getDistance(LEFT);

  
  // Imprimo las distancias recibidas de Tof
  Serial.print("DistanciaF:");
  Serial.println(distancias[FRONT]);
  Serial.print("DistanciaD:");
  Serial.println(distancias[RIGHT]);
  Serial.print("DistanciaI:");
  Serial.println(distancias[LEFT]);

  Serial.print("enc_START: ");
  Serial.println(enc_START);
  Serial.print("enc_END: ");
  Serial.println(enc_END);
  Serial.print("Estado: ");
  Serial.println(run_mode);
  


  // casuística:
  tiempo=millis(); // variable de tiempo que actualizamos en cada bucle

  // Error o callejón sin salida, preocupante
  if (((distancias[FRONT] < 20)||(distancias[RIGHT] < 20)||(distancias[LEFT] < 20))) {
    if(tiempo>=t_backON){
      t_backON=millis()+2000; 
    }
    else{
      // Atras
      run_mode=RUN_MODE_GOBACK;
//      tiempo_obj=millis()+1000;
    }
  }
  else{
    t_backON=millis();
    if(!enc_START && !enc_END){
      // Resetamos los valores de laberinto
      if(distancias[RIGHT]>=200){
        run_mode=RUN_MODE_GORIGHT;
//        tiempo_obj=millis()+T_GIROD;
      }
      else if ((distancias[FRONT] <= 80) && (distancias[FRONT] >= 40) && (distancias[LEFT] >= 200)){
        run_mode=RUN_MODE_GOLEFT;
//        tiempo_obj=millis()+T_GIROI;
      }
      else if (distancias[LEFT] <=200 && (distancias[RIGHT] <= 200) && (distancias[FRONT] <= 80)) {
        run_mode=RUN_MODE_GO180;
//        tiempo_obj=millis()+T_GIRO180;
      }
      else{
        run_mode=RUN_MODE_PID;
//        tiempo_obj=millis();
      }
    }
  }

  switch (run_mode) {
    case RUN_MODE_DEFAULT:
      mismotores.parar();
    break;

    case RUN_MODE_PID:
      mismotores.seguirpared(distancias[RIGHT]);
    break;

    case RUN_MODE_GORIGHT:
      if(!enc_START) enc_END=mismotores.avanzarDistancia(DISTANCIA_FOR);
      else enc_START=mismotores.girar(GIRO90_D);

    break;

    case RUN_MODE_GOLEFT:
      
      if(!enc_START) enc_END=mismotores.avanzarDistancia(DISTANCIA_FOR);
      else enc_START=mismotores.girar(GIRO90_I);
    break;

    case RUN_MODE_GO180:
      enc_START=mismotores.girar(GIRO180_D);
      enc_END=enc_START;
    break;

    case RUN_MODE_GOBACK:
      enc_START=mismotores.avanzarDistancia(DISTANCIA_BACK);
      enc_END=enc_START;
    break;

    case RUN_MODE_STOP:
      mismotores.parar();
    break;
  }

#ifdef PRINT_POSITION

display.clearDisplay();
display.setCursor(0,0);
display.setTextColor(SSD1306_WHITE);
display.setTextSize(1);
display.print("M_right: ");
display.print(control.getEncoders()[RIGHT].getPosicionGrados());
display.print(" M_left: ");
display.print(control.getEncoders()[LEFT].getPosicionGrados());
display.display();

#endif

}

