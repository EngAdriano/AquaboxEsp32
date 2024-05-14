#include "Arduino.h"
#include <WiFi.h>
#include "WiFiManager.h"
#include "EventoSensores.hpp"
#include "Relogio.hpp"
#include "ModuloRele.hpp"

//Protótipo das funções e tasks
void taskSensores (void *params);
void taskReles(void *params);
void taskRelogio( void *params);
void nivelBaixoPressionado();
void nivelAltoPressionado();
void nivelBaixoLiberado();
void nivelAltoLiberado();

void setup() 
{
  /* Inicializa serial (baudrate 115200) */
  Serial.begin(115200);

  /*Executa a conexão com WiFi via WiFiManager*/
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
/* --------------- Tarefas / Funções ----------------*/
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

void taskReles(void *params)
{
    ModuloRele Reles(2, 15, 5, 4, false);

    while(true)
    {
        for (int i = 0; i < N_RELES; i++)
        {
            Reles.toggle(i);
            vTaskDelay( 500 / portTICK_PERIOD_MS ); 

        }
    }
}

void taskRelogio( void *params)
{
    const char* ntpServer = "pool.ntp.org";
    const long  gmtOffset_sec = -14400; //GMT Time Brazil
    const int   daylightOffset_sec = 3600;

    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();

    while(true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printLocalTime();
    }
}
