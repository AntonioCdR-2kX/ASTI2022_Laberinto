#include "NavLaberinto.h"

NavLaberinto::NavLaberinto(Motor* motor_d, Motor* motor_i, Encoder_p* encoder_d, Encoder_p* encoder_i)
{
    misMotores[LEFT] = motor_i;
    misMotores[RIGHT] = motor_d;
    misEncoders[LEFT] = encoder_i;
    misEncoders[RIGHT] = encoder_d;
}

void NavLaberinto::init()
{
    // Inicializacion de las variables de control
    input = 0;
    output = 0;
    setpoint = 0;
    vel_d = 0;
    vel_i = 0;

    vel_base=V_BASE;
    vel_max=V_MAX;
    vel_pid=0;
}

bool NavLaberinto::setPosition(float setpoint_l, float setpoint_r)
{
    float gradosl, gradosr;
    float error_l, error_r;
    float error_l_prev, error_r_prev;
    float error_l_integral = 0.0, error_r_integral = 0.0;
    float error_l_deriv = 0.0, error_r_deriv = 0.0;

    //Obtenemos posicion en este instante
    gradosl = misEncoders[LEFT]->getPosicionGrados();
    gradosr = misEncoders[RIGHT]->getPosicionGrados();

    // Calculamos los errores
    error_l = setpoint_l - gradosl;
    error_r = setpoint_r - gradosr;

    // Se ejecuta el siguiente while hasta que se alcance un error determinado
    // TODO: Deberia haber un timeout para evitar que se quede en un bucle infinito
    if((error_l > 1.0 || error_l< -1.0) || (error_r > 1.0 || error_r < -1.0))
    {
        // Lectura
        gradosl = misEncoders[LEFT]->getPosicionGrados();
        gradosr = misEncoders[RIGHT]->getPosicionGrados();

        // Cálculos
        error_l = setpoint_l - gradosl;
        error_l_integral += error_l;
        error_l_deriv = error_l - error_l_prev;

        error_r = setpoint_r - gradosr;
        error_r_integral += error_r;
        error_r_deriv = error_r - error_r_prev;

        vel_i = kp_EN * error_l + ki_EN * error_l_integral + kd_EN * error_l_deriv;
        vel_d = kp_EN * error_r + ki_EN * error_r_integral + kd_EN * error_r_deriv;


        // Se aplican las velocidades
        // Se hace un control por saturacion
        // Motor Izquierdo
        if (vel_i > 0.0)
        {
            misMotores[LEFT]->setFwd();
            misMotores[LEFT]->setPWM(vel_i > VEL_MAX_ENC? VEL_MAX_ENC : vel_i);
        }
        else{
            misMotores[LEFT]->setBack();
            misMotores[LEFT]->setPWM(-vel_d > VEL_MAX_ENC? VEL_MAX_ENC : -vel_d);
        }

        // Motor Derecho
        if (vel_d > 0.0)
        {
            misMotores[RIGHT]->setFwd();
            misMotores[RIGHT]->setPWM(vel_d > VEL_MAX_ENC? VEL_MAX_ENC : vel_d);
        }
        else
        {
            misMotores[RIGHT]->setBack();
            misMotores[RIGHT]->setPWM(-vel_d > VEL_MAX_ENC? VEL_MAX_ENC : -vel_d);
        }
        return 1;

        // Dejamos los motores libres
//        misMotores[LEFT]->setFree();
//        misMotores[RIGHT]->setFree();
    }
    else
    {
        this->resetEncoders();
        return 0;
    } 
}

bool NavLaberinto::avanzarDistancia(float distance)
{
    float angle = 360.0*distance/(ROBOT_RADIUS*PI);
    return this->setPosition(angle, angle);
}

bool NavLaberinto::girar(float beta)
{
    float phi = ROBOT_WIDTH*beta/ROBOT_RADIUS;
    return this->setPosition(phi, -phi); 
}

void NavLaberinto::resetEncoders()
{
    misEncoders[LEFT]->resetPosicion();
    misEncoders[RIGHT]->resetPosicion();
}

void NavLaberinto::parar()
{
    misMotores[LEFT]->setFree();
    misMotores[RIGHT]->setFree();
}

//Si se sigue la pared
void NavLaberinto::seguirpared(int distancia_D)
{
    // Calculamos los errores
    error_D = distancia_D - D_OPT;
    error_D_integral += error_D;
    error_D_deriv = error_D - error_D_prev;

    error_D_prev = error_D;

    // Calculamos el PID para la pared
    vel_pid = (int) (kp_EN * error_D + ki_EN * error_D_integral + kd_EN * error_D_deriv);

    vel_i = vel_base + vel_pid;
    vel_d = vel_base - vel_pid;


    // Se aplican las velocidades
    // Se hace un control por saturacion
    // Motor Izquierdo
    if (vel_i > 0.0)
    {
        misMotores[LEFT]->setFwd();
        misMotores[LEFT]->setPWM(vel_i > vel_max? vel_max : vel_i);
    }
    else{
        misMotores[LEFT]->setBack();
        misMotores[LEFT]->setPWM(-vel_d > vel_max? vel_max : -vel_d);
    }

    // Motor Derecho
    if (vel_d > 0.0)
    {
        misMotores[RIGHT]->setFwd();
        misMotores[RIGHT]->setPWM(vel_d > VEL_MAX_ENC? VEL_MAX_ENC : vel_d);
    }
    else
    {
        misMotores[RIGHT]->setBack();
        misMotores[RIGHT]->setPWM(-vel_d > VEL_MAX_ENC? VEL_MAX_ENC : -vel_d);
    }

    // Dejamos los motores libres
//    misMotores[LEFT]->setFree();
//    misMotores[RIGHT]->setFree();
}



/*
void NavLaberinto::compute(int ndistancia)
{
    distancia = ndistancia;
    myPID->Input = (distancia - distanciaOPT);
    myPID->Update();
    output = myPID->Output; 
}
*/
/*
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
*/
/*
void NavLaberinto::retroceder()
{
    MisMotores[RIGHT]->setBack();
    MisMotores[LEFT]->setBack();
    MisMotores[RIGHT]->setPWM(vel_base);
    MisMotores[LEFT]->setPWM(vel_base) ;

    vTaskDelay(MILLIS_RETRO / portTICK_PERIOD_MS);
    // puede que haya que poner uno en cada dirección
    
}
*/
/*
void NavLaberinto::avanzar()
{
    MisMotores[RIGHT]->setFwd();
    MisMotores[LEFT]->setFwd();
    MisMotores[RIGHT]->setPWM(vel_base);
    MisMotores[LEFT]->setPWM(vel_base) ;

    vTaskDelay(MILLIS_AVANZ / portTICK_PERIOD_MS);
      // puede que haya que poner uno en cada dirección
    
}
*/
/*
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
*/
/*
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
*/









