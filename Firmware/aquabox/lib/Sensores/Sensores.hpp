#ifndef SENSORES_HPP
#define SENSORES_HPP

#include "Arduino.h"
#include "EventoSensores.hpp"

const int SENSOR_NIVEL_BAIXO = 27;
const int SENSOR_NIVEL_ALTO = 14;

//Protótipo das funções
void nivelBaixoPressionado();
void nivelAltoPressionado();
void nivelBaixoLiberado();
void nivelAltoLiberado();
void taskSensores (void *params);

#endif //SENSORES_HPP