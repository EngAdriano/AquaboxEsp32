#ifndef MODULORELE_H
#define MODULORELE_H

#include "Arduino.h"

const int N_RELES = 4;

class ModuloRele
{
    public:

        ModuloRele(const int pin0, const int pin1, const int pin2, const int pin3, const bool nivelDeAtivacao);

        void set(const int index, const bool estado);
        void on(const int index);
        void off(const int index);
        void toggle(const int index);

        void setAll(const bool estado);
        void onAll(void);
        void offAll(void);
        void toggleAll(void);

    private:

        bool EstadoDeAtivacao;

        int pins[N_RELES];
        bool estado[N_RELES];
};

#endif //MODULORELE_H