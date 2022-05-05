// Objeto encoder.

#ifndef _ENCODER_
#define _ENCODER_
#include<Arduino.h>


class Encoder_p
{
    private:
        int pin_canal_A;
        int pin_canal_B;
        int posicion_tics;
        float posicion_grados;
        float grados_por_tic;

    public:
        Encoder_p(int, int, float);
        void init();
        void actualizar_posicion(); //Llamar con interrupcion en canal A.
        float getPosicionGrados();
        int getTics();
        void resetPosicion(); //Pone la posicion a 0;
        void setPosicionGrados(int);

};

//HOli
#endif
