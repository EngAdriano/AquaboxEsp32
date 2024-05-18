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
void taskControle(void *params);
void taskSensores(void *params);
void taskReles(void *params);
void taskRelogio( void *params);
void btnBombaPressionado();
void btnBombaLiberado();
void nivelBaixoPressionado();
void nivelAltoPressionado();
void nivelBaixoLiberado();
void nivelAltoLiberado();
bool dadoNaFila(int result);

/* filas (queues) */
QueueHandle_t xQueue_Reles, xQueue_Controle;

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
    xQueue_Controle = xQueueCreate(4, sizeof(int));

    //Task de monitoramento e leitura dos sensores de nível
    xTaskCreate(taskRelogio, "Relogio", 2048, NULL, 3, NULL);

    //Task para trabalhos co relógio de tempo real
    xTaskCreate(taskSensores, "Sensores", 2048, NULL, 2, NULL); 

    //Task de acionamento dos relés
    xTaskCreate(taskReles, "Reles", 1024, NULL, 1,NULL);

    //Task de controle
    xTaskCreate(taskControle, "Controle", 2048, NULL, 2, NULL);
}

void loop() 
{
  
}

/* --------------------------------------------------*/
/* --------------- Tarefas / Funções ----------------*/
/* --------------------------------------------------*/

void taskControle(void *params)
{
    /*Regra de negócio ficam aqui, centro de comando*/
    int receive = 0;
    int result = 0;
    bool erro = true;

    while(true)
    {
        /* Espera até algo ser recebido na queue */
        xQueueReceive(xQueue_Controle, (void *)&receive, portMAX_DELAY);

        Serial.print("Comando no controle: ");
        Serial.println (receive);

        if (receive > 99 & receive < 199)
        {
            Serial.println("Enviando dados para relés");
            result = xQueueSend(xQueue_Reles, &receive, 500 / portTICK_PERIOD_MS);
            erro = dadoNaFila(result);
            if(!erro)
            {
                Serial.println("Erro ao colocar dado na fila");
                erro = true;
            }
        }
    }
}

void taskSensores(void *params)
{
    /* Estânciar objetos*/
    EventoSensores nivelBaixo(SENSOR_NIVEL_BAIXO, LOW);
    EventoSensores nivelAlto(SENSOR_NIVEL_ALTO, LOW);
    EventoSensores btnBomba(BOMBA, LOW);
    EventoSensores btnManutencao(MANUTENCAO, LOW);

    nivelBaixo.setPressionadoCallback(&nivelBaixoPressionado);
    nivelAlto.setPressionadoCallback(&nivelAltoPressionado);
    nivelBaixo.setLiberadoCallback(&nivelBaixoLiberado);
    nivelAlto.setLiberadoCallback(&nivelAltoLiberado);
    btnBomba.setPressionadoCallback(&btnBombaPressionado);
    btnBomba.setLiberadoCallback(&btnBombaLiberado);

    while(true)
    {
        nivelBaixo.process();
        nivelAlto.process();
        btnBomba.process();

        /* Espera um segundo */
        vTaskDelay( 100 / portTICK_PERIOD_MS ); 
    }
}

void btnBombaPressionado()
{
    int result = 0;
    int comando = LIGA_BOMBA;
    bool erro = true;

    Serial.println("Botão da Bomba pressionado");

    result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
    erro = dadoNaFila(result);
    if(!erro)
    {
        Serial.println("Erro ao colocar dado na fila");
        erro = true;
    }
}

void btnBombaLiberado()
{
    int result = 0;
    int comando = DESLIGA_BOMBA;
    bool erro = true;

    result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
    erro = dadoNaFila(result);
    if(!erro)
    {
        Serial.println("Erro ao colocar dado na fila");
        erro = true;
    }
}

void nivelBaixoPressionado()
{
    int result = 0;
    int comando = LIGA_CAIXA;
    bool erro = true;

    result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
    erro = dadoNaFila(result);
    if(!erro)
    {
        Serial.println("Erro ao colocar dado na fila");
        erro = true;
    }
}

void nivelAltoPressionado()
{
    int result = 0;
    int comando = DESLIGA_CAIXA;
    bool erro = true;

    result = xQueueSend(xQueue_Reles, &comando, 500 / portTICK_PERIOD_MS);
    erro = dadoNaFila(result);
    if(!erro)
    {
        Serial.println("Erro ao colocar dado na fila");
        erro = true;
    }
}

void nivelBaixoLiberado()
{
    //Tratar depois para detectar erro no sensor
}

void nivelAltoLiberado()
{
    //Tratar depois para detectar erro no sensor
}

bool dadoNaFila(int result)
{
    if(result)      //Checar o retorno para verificar se o dado foi inserido na fila com sucesso
    {
        return true;
    }
    else
    {
        return false;
    }
}

void taskReles(void *params)
{
    int receive = 0;

    const int RELAYS[N_RELES] = {RELE_BOMBA, RELE_CAIXA, RELE_SETOR_1, RELE_SETOR_2};
    ModuloRele Reles(RELAYS[0], RELAYS[1], RELAYS[2], RELAYS[3], true);
    Reles.offAll();

    while(true)
    {
        /* Espera até algo ser recebido na queue */
        xQueueReceive(xQueue_Reles, (void *)&receive, portMAX_DELAY);
        Serial.print("Comando nos relés: ");
        Serial.println (receive);

        switch (receive)
        {
        case 100:               //Desliga todos os relés
            Reles.offAll();
            receive = 0;
            break;

        case 110:               //desliga somente a bomba               
            Reles.off(0);
            receive = 0;
            break;

        case 111:               //Liga somente a bomba                
            Reles.on(0);
            receive = 0;
            break;

        case 120:               //desliga o encher da caixa d'água                                            
            Reles.off(0);
            vTaskDelay( 3000 / portTICK_PERIOD_MS ); 
            Reles.off(1);
            receive = 0;
            break;

        case 121:               //liga o encher da caixa d'água                               
            Reles.on(1);
            vTaskDelay( 3000 / portTICK_PERIOD_MS );
            Reles.on(0);
            receive = 0;
            break;

        case 130:               //desliga a irrigação do setor 1                              
            Reles.off(0);
            vTaskDelay( 2000 / portTICK_PERIOD_MS ); 
            Reles.off(2);
            receive = 0;
            break;

        case 131:               //liga a irrigação do setor 1
            Reles.on(2);
            vTaskDelay( 2000 / portTICK_PERIOD_MS );
            Reles.on(0);
            receive = 0;
            break;

        case 140:               //desliga a irrigação do setor 2
            Reles.off(0);
            vTaskDelay( 2000 / portTICK_PERIOD_MS ); 
            Reles.off(3);
            receive = 0;
            break;

        case 141:               //liga a irrigação do setor 2
            Reles.on(3);
            vTaskDelay( 2000 / portTICK_PERIOD_MS );
            Reles.on(0);
            receive = 0;
            break;
        
        default:
            break;
        }
    }
}

void taskRelogio( void *params)
{
    /* Estrutura para dias e horários para irrigação*/
    struct horarioDeIrrigacao
    {
        int horaDeInicio;
        int minutoDeInicio;
        int tempoDeDuracao;
    };

    /* Vetor dos dias da semana*/
    bool dias[7] = {true, true, true, true, true, true, true};

    struct horarioDeIrrigacao horarioSetor1;
    struct horarioDeIrrigacao horarioSetor2;
    
    /* Configuração Setor 1*/
    horarioSetor1.horaDeInicio = 17;
    horarioSetor1.minutoDeInicio = 00;
    horarioSetor1.tempoDeDuracao = 20;

    /* Configuração Setor */
    horarioSetor2.horaDeInicio = 17;
    horarioSetor2.minutoDeInicio = 30;
    horarioSetor2.tempoDeDuracao = 20;

    /* Variáveis para o relógio*/
    const char* ntpServer = "pool.ntp.org";
    const long  gmtOffset_sec = -14400; //GMT Time Brazil
    const int   daylightOffset_sec = 3600;
    int diaDaSemana = 0;

    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;

    while(true)
    {
        if(!getLocalTime(&timeinfo))
        {
            Serial.println("Falha ao obter o relógio");
        }
        else
        {
            diaDaSemana = timeinfo.tm_wday;

            if(dias[diaDaSemana] == true)
            {
                //Serial.println("Tem que ligar a Irrigação");
                //Serial.println(diaDaSemana);

                if((horarioSetor1.horaDeInicio = timeinfo.tm_hour) & (horarioSetor1.minutoDeInicio = timeinfo.tm_min))
                {
                    
                }
            }
            else
            {
                Serial.println("Não precisa ligar");
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


