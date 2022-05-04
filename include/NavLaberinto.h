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
        int distancia;
        float output;

        //Array con los dos motores a controlar
        Motor* MisMotores[2];
        //Array con los dos encoders 
        Encoder* MisEncoders[2];

        //Pid
        PID::PIDController<float>* myPID ;
        PID::PIDParameters<float> parametros_PID;
        
        //Parametros de la velocidad
        uint8_t vel_base; 
        uint8_t vel_max;
        uint8_t vel_pid;

    public:

        //Constructor
        NavLaberinto(Motor*, Motor*);    
        void setVelBase(uint8_t a){vel_base = a;};
        void setPIDparam(PID::PIDParameters<float> a)
            {
                parametros_PID = a;
                myPID->SetTunings(a);
                };
        void parar();
        void avanzar();
        void girar(bool);
        void retroceder();
        
        float getOutput();
        float getPos(){return myPID->Input;};
        float getKp(){return myPID->GetKp();};
        float getKd(){return myPID->GetKd();};
        float getKi(){return myPID->GetKi();};
        
        void compute(int);
        void giro90(bool);
        void seguirpared();

        void encompute();
        void 

};

#endif