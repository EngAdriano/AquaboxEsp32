#include "Sensores.hpp"

EventoSensores nivelBaixo(SENSOR_NIVEL_BAIXO, LOW);
EventoSensores nivelAlto(SENSOR_NIVEL_ALTO, LOW);

void taskSensores (void *params)
{
    nivelBaixo.setPressionadoCallback(&nivelBaixoPressionado);
    nivelAlto.setPressionadoCallback(&nivelAltoPressionado);
    nivelBaixo.setLiberadoCallback(&nivelBaixoLiberado);
    nivelAlto.setLiberadoCallback(&nivelAltoLiberado);

    while(true)
    {
        nivelBaixo.process();
        nivelAlto.process();

        /* Espera um segundo */
        vTaskDelay( 100 / portTICK_PERIOD_MS ); 


    }
}

void nivelBaixoPressionado()
{
    Serial.println("Pressionou o sensor de nivel BAIXO");
}

void nivelAltoPressionado()
{
    Serial.println("Pressionou o sensor de nivel ALTO");
}

void nivelBaixoLiberado()
{
    Serial.println("Liberado o sensor de nivel BAIXO");
}

void nivelAltoLiberado()
{
    Serial.println("Liberado o sensor de nivel ALTO");
}
