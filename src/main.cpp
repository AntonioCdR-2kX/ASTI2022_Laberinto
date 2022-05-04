#include <Arduino.h>

//Cosas que hacer:
/*
f1 -> Arduino: Board config -> ESP32 Dev Module
f1 -> Arduino: Initialize (solo una vez)
f1 -> Arduino: Verify (compilar)
*/

#include "funcionesSetup.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "TofSensors.h"
#include "funcionesSetup.h"
#include "PCF8575.h"
#include "Pinout.h"
#include "Config.h"
#include "Motor.h"
#include "Encoder.h"
#include "NavLaberinto.h"
#include "Logo.h"

//------------------------- EL codigo  -------------------------------------------

// VARIABLES

// Maquina de estado para las fases del laberinto
typedef enum {RUN_MODE_DEFAULT, RUN_MODE_PID, RUN_MODE_GORIGHT, RUN_MODE_GOLEFT, RUN_MODE_GO180, RUN_MODE_GOBACK, RUN_MODE_STOP} runmode_t;
runmode_t run_mode = RUN_MODE_DEFAULT;

//Variables de la distancia de cada sensor
int distancias[3];
int distanciaC;
int contador = 0; // solo para el inicio

//Variables globales de tiempo
int tiempo=0;
int tiempo_obj=0;
int t_backON=0;


//#include "FuncionesDisplay.h"


 //Objeto global para el extensor de pines
PCF8575 pinExtensor =  PCF8575(0x20);

//objeto global para el display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); 

//Objeto que maneja los 3 sensores tof
TofSensors MySensors(1, 2, 3, &pinExtensor); 

//Objetos para los dos motores (NO control de posicion)
Motor Motor_derecho(PIN_MOTOR_D_IN1, PIN_MOTOR_D_IN2, PIN_MOTOR_D_PWM, PWM_CH_D, PWM_FREC, PWM_RES); //Motor derecho
Motor Motor_izquierdo(PIN_MOTOR_I_IN1, PIN_MOTOR_I_IN2, PIN_MOTOR_I_PWM, PWM_CH_I, PWM_FREC, PWM_RES); //Motor izquierdo

//Objetos para control de posicion (encoders)
Encoder Encoder_derecho(PIN_ENCODER_D_A, PIN_ENCODER_D_B, float tics_vuelta);
Encoder Encoder_izquierdo();

// Objeto que maneja los motores
NavLaberinto mismotores(&Motor_derecho, &Motor_izquierdo);

// Movidas del multitasking
TaskHandle_t Task_CORE0; // Nucleo secundario
TaskHandle_t Task_CORE1; // Nucleo principal

void setup(){

//    Wire.setClock(800000); //Igual esto es lo que fuerza la velocidad del reloj

    // Inicializamos el serial
    Serial.begin(9600);

    // Se inicializa y configura el display
    configDisplay();
    
    display.clearDisplay();
    display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_reset, LOGO_WIDTH, LOGO_HEIGHT, 1);
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(1, 1);
    display.println("Configurando Hardware...");
    display.display();

    //Configuro las interrupciones
    attachInterrupt(digitalPinToInterrupt(PIN_MOTORA_CANALA), handler_encoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_MOTORB_CANALA), handler_encoderB, CHANGE);


    // Se inicializan los pines de los motores
    Motor_derecho.init();
    Motor_izquierdo.init();
    // Se inicializan los pines de los tof
    MySensors.init();
    // Se inicializa el extensor de pines
    pinExtensor.begin();
    // Se cambia la direccion de memoria de los sensores tof y se bootean
    MySensors.setID(TOF_ADRESS_FRONT, TOF_ADRESS_RIGHT, TOF_ADRESS_LEFT);
    
    display.println(" Listo");
    display.display();

//    display.println(" Listo"); // Se repite?
//    display.display();

    delay(1000);
    display.clearDisplay();

    // -------------- Creamos los dos hilos en los dos nucleos ----------------

    // Para el core 0 (secundario)
    xTaskCreatePinnedToCore(
        TaskCORE0code, // funcion de la task
        "Task_CORE0",  // nombre de la task
        10000,         // tamano del stack de la task
        NULL,          // Parametos
        1,             // task priority
        &Task_CORE0,   // callback
        0              // se lo mandamos al nucleo 0
    );

    // Para el core 1 (principal)
    xTaskCreatePinnedToCore(
        TaskCORE1code, // funcion de la task
        "Task_CORE1",  // nombre de la task
        10000,         // tamano del stack de la task
        NULL,          // Parametos
        1,             // task priority
        &Task_CORE1,   // callback
        1              // se lo mandamos al nucleo 1
    );
}




// El core 0 lo usaremos para manejar la movida web
//  y alguna tarea secundaria
void TaskCORE0code(void *pvParameters)
{ 
    int lineas_a_contar = 0;
    bool flag_horizontal = false;
    uint8_t numero_de_veces_giradas = 0;
    Serial.println("Core 0 activo");
    display.println("Core 0 activo");
    // Bucle infinito 
    // AQUÍ PROGRAMO YO
    for (;;)
    {
        /*
        // solo para entrar en el laberinto
        if(contador == 0){
            misMotores.avanzar();
            //delay(2000); // que alguien cambie esta basura
        }
        */
        // leo sensores
        if(MySensors.getDistance(FRONT)!=-1) distancias[FRONT] = MySensors.getDistance(FRONT);
        if(MySensors.getDistance(RIGHT)!=-1) distancias[RIGHT] = MySensors.getDistance(RIGHT);
        if(MySensors.getDistance(LEFT)!=-1) distancias[LEFT] = MySensors.getDistance(LEFT);

        /*
        // Imprimo las distancias recibidas de Tof
        Serial.print("DistanciaF:");
        Serial.println(distanciaF);
        Serial.print("DistanciaD:");
        Serial.println(distanciaD);
        Serial.print("DistanciaI:");
        Serial.println(distanciaI);
        */


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
                tiempo_obj=millis()+1000;
            }
        }
        else{
            t_backON=millis();
            if(tiempo>=tiempo_obj){
                if(distancias[RIGHT]>=100){
                    run_mode=RUN_MODE_GORIGHT;
                    tiempo_obj=millis()+T_GIROD;
                }
                else if ((distancias[FRONT] <= 40) && (distancias[FRONT] >= 20) && (distancias[LEFT] >= 100)){
                    run_mode=RUN_MODE_GOLEFT;
                    tiempo_obj=millis()+T_GIROI;
                }
                else if (distancias[LEFT] <=100 && (distancias[RIGHT] <= 100) && (distancias[FRONT] <= 40)) {
                    run_mode=RUN_MODE_GO180;
                    tiempo_obj=millis()+T_GIRO180;
                }
                else{
                    run_mode=RUN_MODE_PID;
                    tiempo_obj=millis();
                }

            }
        }

        switch (run_mode) {
            case RUN_MODE_DEFAULT:
                mismotores.parar();

            break;
            case RUN_MODE_PID:
                mismotores.seguirpared();

            break;
            case RUN_MODE_GORIGHT:
                mismotores.giro90(ANTIHORARIO);
        
            break;
            case RUN_MODE_GOLEFT:
                mismotores.giro90(HORARIO);

            break;
            case RUN_MODE_GO180:
                mismotores.girar(ANTIHORARIO);
    
            break;
            case RUN_MODE_GOBACK:
                mismotores.retroceder();
    
            break;
            case RUN_MODE_STOP:
                mismotores.parar();
    
            break;
        }
    }
}

// El core 1 contiene cosas secundarias
void TaskCORE1code(void *pvParameters)
{
    Serial.println("Core 1 activo");
    display.println("Core 1 activo");
    // Bucle infinito // AQUÍ PROGRAMO YO (wifi)
    for (;;)
    {
        //displayThings();
        mismotores.compute(distancias[RIGHT]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// No usado
void loop(void)
{
   
}

void handler_encoderA()
{
  myPullup.getEncoder(A).actualizar_posicion();  
}
void handler_encoderB()
{
  myPullup.getEncoder(B).actualizar_posicion();
}