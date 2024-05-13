#include "ModuloRele.hpp"

ModuloRele::ModuloRele(const int pin0, const int pin1, const int pin2, const int pin3, const bool nivelDeAtivacao):
    EstadoDeAtivacao(nivelDeAtivacao),
    pins{pin0, pin1, pin2, pin3},
    estado{false, false, false, false}
{
    for (int i = 0; i < N_RELES; i++)
    {
        pinMode(pins[i], OUTPUT);
    }
}

void ModuloRele::set(const int index, const bool estado)
{
    bool s;

    s = estado ^ (!EstadoDeAtivacao);

    this->estado[index] = s;
    digitalWrite(pins[index], EstadoDeAtivacao);
}
void ModuloRele::on(const int index)
{
    digitalWrite(pins[index], EstadoDeAtivacao);
}
void ModuloRele::off(const int index)
{
    estado[index] = !EstadoDeAtivacao;
    digitalWrite(pins[index], !EstadoDeAtivacao);
}
void ModuloRele::toggle(const int index)
{
    estado[index] = !estado[index];
    digitalWrite(pins[index], estado[index]);
}

void ModuloRele::setAll(const bool estado)
{
    for (int i = 0; i < N_RELES; i++)
    {
        set(i, estado);
    }
}
void ModuloRele::onAll(void)
{
    for (int i = 0; i < N_RELES; i++)
    {
        on(i);
    }
}
void ModuloRele::offAll(void)
{
    for (int i = 0; i < N_RELES; i++)
    {
        off(i);
    }
}
void ModuloRele::toggleAll(void)
{
    for (int i = 0; i < N_RELES; i++)
    {
        toggle(i);
    }
}