#include "Arduino.h"
#include <WiFi.h>
#include "WiFiManager.h"
#include "EventoSensores.hpp"
#include "Relogio.hpp"
#include "ModuloRele.hpp"

/* Pinos*/
#define BOMBA               36
#define MANUTENCAO          39
#define SENSOR_NIVEL_BAIXO  27
#define SENSOR_NIVEL_ALTO   14
#define RELE_BOMBA          2
#define RELE_CAIXA          15
#define RELE_SETOR_1        5
#define RELE_SETOR_2        4

/* Comandos*/
#define DESLIGA_RELES   100         //Desliga todos os relés
#define DESLIGA_BOMBA   110         //Desliga o motor da bomba d'água
#define LIGA_BOMBA      111         //Liga o motor da bomba d'água
#define DESLIGA_CAIXA   120         //Desliga o enchimento da caixa d'água
#define LIGA_CAIXA      121         //Liga o enchimento da caixa d'água
#define DESLIGA_SETOR1  130         //desliga a inrrigação do setor 1
#define LIGA_SETOR1     131         //Liga a inrrigação do setor 1
#define DESLIGA_SETOR2  140         //desliga a inrrigação do setor 2
#define LIGA_SETOR2     141         //Liga a inrrigação do setor 2

//Protótipo das funções e tasks
void taskSensores (void *params);
void taskReles(void *params);
void taskRelogio( void *params);
void nivelBaixoPressionado();
void nivelAltoPressionado();
void nivelBaixoLiberado();
void nivelAltoLiberado();

/* filas (queues) */
QueueHandle_t xQueue_Reles, xQueue_Comandos;

/* semaforos utilizados */

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
    
    //Criação da fila (Queue)
    ////Ao inicializar a fila devemos passar o tamanho dela e o tipo de dado. Pode ser inclusive estruturas
    xQueue_Reles = xQueueCreate(4, sizeof(int));

    //Task de monitoramento e leitura dos sensores de nível
    xTaskCreate(taskRelogio, "Relogio", 2048, NULL, 3, NULL);

    //Task para trabalhos co relógio de tempo real
    xTaskCreate(taskSensores, "Sensores", 2048, NULL, 2, NULL); 

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
    /* Estânciar objetos*/
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
    
    int result = 0;
    Serial.println("Pressionou o sensor de nivel BAIXO");
    
    int comando = LIGA_CAIXA;

    result = xQueueSend(xQueue_Reles, &comando, 500 / portTICK_PERIOD_MS);

    if(result)      //Checar o retorno para verificar se o dado foi inserido na fila com sucesso
        {
            printf("Dado inserido na fila com sucesso \n");
        }
        else
        {
            printf("Erro ao inserir dado na fila \n");
        }
    /* Espera um segundo */
        vTaskDelay( 1000 / portTICK_PERIOD_MS ); 
}

void nivelAltoPressionado()
{
    int result = 0;
    Serial.println("Pressionou o sensor de nivel ALTO");
    
    int comando = DESLIGA_CAIXA;

    result = xQueueSend(xQueue_Reles, &comando, 500 / portTICK_PERIOD_MS);

    if(result)      //Checar o retorno para verificar se o dado foi inserido na fila com sucesso
        {
            printf("Dado inserido na fila com sucesso \n");
        }
        else
        {
            printf("Erro ao inserir dado na fila \n");
        }
    /* Espera um segundo */
        vTaskDelay( 1000 / portTICK_PERIOD_MS ); 
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
    const int RELAYS[N_RELES] = {RELE_BOMBA, RELE_CAIXA, RELE_SETOR_1, RELE_SETOR_2};
    ModuloRele Reles(RELAYS[0], RELAYS[1], RELAYS[2], RELAYS[3], true);
    Reles.offAll();

    int static receive = 0;

    while(true)
    {
        /* Espera até algo ser recebido na queue */
        xQueueReceive(xQueue_Reles, (void *)&receive, portMAX_DELAY);
        Serial.println("Dado recebido: ");
        Serial.println(receive);

        switch (receive)
        {
        case 100:               //Desliga todos os relés
            Reles.offAll();
            Serial.println("Relés Desligados");
            break;

        case 110:               //desliga somente a bomba               
            Reles.off(0);
            break;

        case 111:               //Liga somente a bomba                
            Reles.on(0);
            break;

        case 120:               //desliga o encher da caixa d'água                                            
            Reles.off(0);
            vTaskDelay( 3000 / portTICK_PERIOD_MS ); 
            Reles.off(1);
            receive = 0;
            break;

        case 121:               //liga o encher da caixa d'água                               
            Serial.println("Executando switch");
            Reles.on(1);
            vTaskDelay( 3000 / portTICK_PERIOD_MS );
            Reles.on(0);
            receive = 0;
            Serial.println("Estou depois");
            break;

        case 130:               //desliga a irrigação do setor 1                              
            Reles.off(0);
            vTaskDelay( 2000 / portTICK_PERIOD_MS ); 
            Reles.off(2);
            break;

        case 131:               //liga a irrigação do setor 1
            Reles.on(2);
            vTaskDelay( 2000 / portTICK_PERIOD_MS );
            Reles.on(0);
            break;

        case 140:               //desliga a irrigação do setor 2
            Reles.off(0);
            vTaskDelay( 2000 / portTICK_PERIOD_MS ); 
            Reles.off(3);
            break;

        case 141:               //liga a irrigação do setor 2
            Reles.on(3);
            vTaskDelay( 2000 / portTICK_PERIOD_MS );
            Reles.on(0);
            break;
        
        default:
            break;
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
