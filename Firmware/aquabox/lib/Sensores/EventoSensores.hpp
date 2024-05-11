#ifndef EVENTOSENSORES_HPP
#define EVENTOSENSORES_HPP

#include "Arduino.h"

class EventoSensores
{
    public:
        EventoSensores(const int pin, const bool nivelPressionado);

        void setPressionadoCallback(void (*callback)(void));
        void setLiberadoCallback(void (*callback)(void));

        void process(void);

    private:
        
        int pin;
        bool nivel;

        bool estadoAnterior;

        void (*pressionado)(void);
        void (*liberado)(void);

};

#endif //EVENTOSENSORES_HPP