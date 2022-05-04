#include "Encoder.h"


Encoder::Encoder(int canal_A, int canal_B, float tics_vuelta)
{
    pin_canal_A = canal_A;
    pin_canal_B = canal_B;
    grados_por_tic = 360.0/(tics_vuelta);
    posicion_tics = 0;
    
}

void Encoder::init()
{
    pinMode(pin_canal_A, INPUT_PULLUP);
    pinMode(pin_canal_B, INPUT_PULLUP);
}

void Encoder::actualizar_posicion()
{
    if(digitalRead(pin_canal_A) == HIGH)
    {
        if(digitalRead(pin_canal_B) == LOW)
            posicion_tics++;
        else
            posicion_tics--;
    }

    /*
    else
    {
        if(digitalRead(pin_canal_B) == LOW)
            posicion_tics--;
        else
            posicion_tics++;
    }

    */

}


float Encoder::getPosicionGrados()
{
    return (grados_por_tic*getTics());
}

int Encoder::getTics()
{
    int pos;
    ATOMIC()
    {
        pos = posicion_tics;
    }
    return posicion_tics;
}

void Encoder::resetPosicion()
{
    posicion_tics = 0;
}

void Encoder::setPosicionGrados(int grados)
{
    posicion_tics = grados/grados_por_tic;
}