#include "EventoSensores.hpp"

EventoSensores::EventoSensores(const int pin, const bool nivelPressionado):
    pin(pin),
    nivel(nivelPressionado)
{
    pinMode(this->pin, INPUT);
    estadoAnterior = digitalRead(this->pin);
}

void EventoSensores::setPressionadoCallback(void (*callback)(void))
{
    pressionado = callback;
}

void EventoSensores::setLiberadoCallback(void (*callback)(void))
{
    liberado = callback;
}
      
void EventoSensores::process(void)
{
    bool estado;

    estado = digitalRead(pin);
    vTaskDelay(20 / portTICK_PERIOD_MS);

    if (estado != estadoAnterior)
    {
        if( estado == nivel)
        {
            if (pressionado != NULL) pressionado();
        }
        else
        {
            if (liberado != NULL) liberado();
        }

        estadoAnterior = estado;
    }
}