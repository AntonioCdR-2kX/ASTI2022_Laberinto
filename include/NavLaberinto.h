#ifndef _NAVLABERINTO_H_
#define _NAVLABERINTO_H_

#include "Config.h"
#include "Motor.h"
#include "Encoder.h"

#define HORARIO 1
#define ANTIHORARIO 0

//Clase que gestiona la navegacion en la prueba de la cuadricula
class NavLaberinto
{
    private:
        //Variables para el PID pared
        float error_D;
        float error_D_prev;
        float error_D_integral = 0.0;
        float error_D_deriv = 0.0;

        //velocidad de los motores
        int vel_d, vel_i; 

        //variables de control
        double input, output, setpoint;

        //Array con los dos motores a controlar
        Motor* misMotores[2];
        //Array con los dos encoders 
        Encoder_p* misEncoders[2];

        //Pid
//        PID::PIDController<float>* myPID ;
//        PID::PIDParameters<float> parametros_PID;
        
        //Parametros de la velocidad
        int vel_base; 
        int vel_max;
        int vel_pid;

    public:

        //Constructor
        NavLaberinto(Motor*, Motor*, Encoder_p*, Encoder_p*);  

// Inicializa el pid y algunas cosas mas
        void init();

        // Avanza una distancia determinada
        // (metodo bloqueante)
        bool avanzarDistancia(float );
        
        // Gira X grados sobre sí mismo
        // (metodo bloqueante)
        bool girar(float beta);

        // Setea la posicion de los dos motores independientemente
        // Me devuelve 1 si ha llegado y 0 si no es así
        bool setPosition(float setpoint_l, float setpoint_r);

        // Resetea la posicion relativa de los encoders
        void resetEncoders();
    
        //Para el robot
        void parar();

        // ######### Geters #########

        // Devuelve array de encoders
        Encoder_p*& getEncoders(){return *misEncoders;};

        // Devuelve array de motores
        Motor*& getMotores(){return *misMotores;};

        int getVelI(){return vel_i;};

        int getVelD(){return vel_d;};

        void seguirpared(int midistancia_D);




};

#endif