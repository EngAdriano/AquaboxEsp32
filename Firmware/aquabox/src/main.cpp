#include "Arduino.h"
#include <WiFi.h>
#include "WiFiManager.h"
#include "Sensores.hpp"
#include "Relogio.hpp"
#include "Reles.hpp"

//Protótipo das funções

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

