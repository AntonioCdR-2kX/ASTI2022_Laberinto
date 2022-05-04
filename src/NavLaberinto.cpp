#include "NavLaberinto.h"

NavLaberinto::NavLaberinto(Motor* b, Motor* c )
{
    
    MisMotores[RIGHT] = b;
    MisMotores[LEFT] = c;

    PID::PIDParameters<float> aux(1.0,0.0,0.0);
    parametros_PID = aux;
    myPID = new PID::PIDController<float>(aux);

    vel_max = 100;
    vel_base = 50;
    myPID->Setpoint = 0.0;
    myPID->SetOutputLimits(-50, 50);
    myPID->TurnOn();
    
}

void NavLaberinto::parar()
{
    int i;
    for(i = 0; i< 2; i++)
    {
        MisMotores[i]->setPWM(0);
        MisMotores[i]->setStop();
    }

    vTaskDelay(MILLIS_PARAR / portTICK_PERIOD_MS);
    // puede que haya que poner uno en cada dirección
    
}

void NavLaberinto::retroceder()
{
    MisMotores[RIGHT]->setBack();
    MisMotores[LEFT]->setBack();
    MisMotores[RIGHT]->setPWM(vel_base);
    MisMotores[LEFT]->setPWM(vel_base) ;

    vTaskDelay(MILLIS_RETRO / portTICK_PERIOD_MS);
    // puede que haya que poner uno en cada dirección
    
}

void NavLaberinto::avanzar()
{
    MisMotores[RIGHT]->setFwd();
    MisMotores[LEFT]->setFwd();
    MisMotores[RIGHT]->setPWM(vel_base);
    MisMotores[LEFT]->setPWM(vel_base) ;

    vTaskDelay(MILLIS_AVANZ / portTICK_PERIOD_MS);
      // puede que haya que poner uno en cada dirección
    
}

void NavLaberinto::girar(bool sentido)
{
    if(sentido == HORARIO)
    {
        MisMotores[RIGHT]->setFwd();
        MisMotores[LEFT]->setBack();
        MisMotores[RIGHT]->setPWM(vel_base);
        MisMotores[LEFT]->setPWM(vel_base) ;
    }else
    {
        MisMotores[RIGHT]->setBack();
        MisMotores[LEFT]->setFwd();
        MisMotores[RIGHT]->setPWM(vel_base);
        MisMotores[LEFT]->setPWM(vel_base) ;
    }
}

void NavLaberinto::giro90(bool sentido)
{
    if(sentido == HORARIO)
    {
        MisMotores[RIGHT]->setFwd();
        MisMotores[LEFT]->setBack();
        MisMotores[RIGHT]->setPWM(vel_base);
        MisMotores[LEFT]->setPWM(vel_base) ;
    }else
    {
        MisMotores[RIGHT]->setBack();
        MisMotores[LEFT]->setFwd();
        MisMotores[RIGHT]->setPWM(vel_base);
        MisMotores[LEFT]->setPWM(vel_base) ;
    }

    vTaskDelay(MILLIS_GIRO90 / portTICK_PERIOD_MS); // puede que haya que poner uno en cada dirección
    
}

void NavLaberinto::seguirpared()
{
    float vel_a, vel_b;

    vel_a = vel_base - output;
    vel_b = vel_base + output;

    MisMotores[RIGHT] ->setFwd();
    MisMotores[LEFT]  ->setFwd();

    vel_a > vel_max? MisMotores[RIGHT]->setPWM(vel_max) : MisMotores[RIGHT]-> setPWM(vel_a);
    vel_b > vel_max? MisMotores[LEFT]->setPWM(vel_max) :MisMotores[LEFT]-> setPWM(vel_b);


}

void NavLaberinto::compute(int ndistancia)
{
    distancia = ndistancia;
    myPID->Input = (distancia - distanciaOPT);
    myPID->Update();
    output = myPID->Output; 
}

void NavLaberinto::encompute()
{
    
}






