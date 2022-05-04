#include "FuncionesDisplay.h"

extern volatile modos Estado;
extern volatile modos_display Estado_display;
extern volatile int inicio_;
extern volatile int final_;
extern TofSensors MySensors;
extern Adafruit_SSD1306 display;
extern QTRSensors qtr;
extern NavLaberinto misMotores;
extern const unsigned char logo_reset [];
extern int caso;

void displayThings()
{
    display.clearDisplay();
        if(Estado_display!= DISPLAY_APAGADO)
        {
            display.setTextColor(SSD1306_WHITE);
            display.setTextSize(1);
            display.setCursor(1, 1);
            switch(Estado_display)
            {
                case DISPLAY_MOSTRAR_ESTADO:
               
                case DISPLAY_MOSTRAR_TOF:
                    display.println("Sensores laser: ");
                    display.println("");
                    display.print("Front: ");
                    display.println(MySensors.getDistance(FRONT));
                    display.println();
                    display.print("Left: ");
                    display.println(MySensors.getDistance(LEFT));
                    display.println();
                    display.print("Right: ");
                    display.println(MySensors.getDistance(RIGHT));
                    display.println();

                break;

                case DISPLAY_MOSTRAR_PARAMETROSPID:
                    display.println("Parametros PID: ");
                    display.println("");
                    display.print("Kp actual:" );
                    display.println(misMotores.getKp());
                    display.print("Ki actual:" );
                    display.println(misMotores.getKi());
                    display.print("Kd actual:" );
                    display.println(misMotores.getKd());
                    display.print("PID input:" );
                    display.println(misMotores.getPos());
                    display.print("PID output: ");
                    display.println(misMotores.getOutput());
                    
                break;

                case DISPLAY_MOSTRAR_LOGO:
                display.clearDisplay();
                display.drawBitmap(
                (display.width()  - LOGO_WIDTH ) / 2,
                (display.height() - LOGO_HEIGHT) / 2,
                logo_reset, LOGO_WIDTH, LOGO_HEIGHT, 1);
                display.display();
                break;

                

            }
        }display.display();
}
