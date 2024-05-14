#include "Arduino.h"
#include <WiFi.h>
#include "WiFiManager.h"
#include "EventoSensores.hpp"
#include "Relogio.hpp"
#include "Reles.hpp"

//Protótipo das funções e tasks
void taskSensores (void *params);
void nivelBaixoPressionado();
void nivelAltoPressionado();
void nivelBaixoLiberado();
void nivelAltoLiberado();

void setup() 
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  WiFiManager wm;

  bool res;

  res = wm.autoConnect("Aquabox");

  if(!res)
  {
    Serial.println("Falha ao conectar");
  }
  else
  {
    Serial.println("Conectado...");
  }
  
  //Task de monitoramento e leitura dos sensores de nível
  xTaskCreate(taskRelogio, "Relogio", 2048, NULL, 3, NULL);

  //Task para trabalhos co relógio de tempo real
  xTaskCreate(taskSensores, "Sensores", 1024, NULL, 2, NULL); 

  //Task de acionamento dos relés
  xTaskCreate(taskReles, "Reles", 1024, NULL, 1,NULL);
}

void loop() 
{
  
}

/* --------------------------------------------------*/
/* ---------------------- Tarefas -------------------*/
/* --------------------------------------------------*/

void taskSensores (void *params)
{

  #define SENSOR_NIVEL_BAIXO 27
  #define SENSOR_NIVEL_ALTO 14
  //const int SENSOR_NIVEL_BAIXO = 27;
  //const int SENSOR_NIVEL_ALTO = 14;

  EventoSensores nivelBaixo(SENSOR_NIVEL_BAIXO, LOW);
  EventoSensores nivelAlto(SENSOR_NIVEL_ALTO, LOW);

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


