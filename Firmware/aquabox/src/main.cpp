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
#define DESLIGA_RELES       100         //Desliga todos os relés  
#define DESLIGA_BOMBA       110         //Desliga o motor da bomba d'água
#define LIGA_BOMBA          111         //Liga o motor da bomba d'água
#define DESLIGA_CAIXA       120         //Desliga o enchimento da caixa d'água
#define LIGA_CAIXA          121         //Liga o enchimento da caixa d'água
#define DESLIGA_SETOR1      130         //desliga a inrrigação do setor 1
#define LIGA_SETOR1         131         //Liga a inrrigação do setor 1
#define DESLIGA_SETOR2      140         //desliga a inrrigação do setor 2
#define LIGA_SETOR2         141         //Liga a inrrigação do setor 2
#define DESLIGA_MANUTENCAO  200         //Desliga manutenção
#define LIGA_MANUTENCAO     201         //Liga manutenção
#define SETOR1_LIGA         10          //Liga alguma função - Setor 1
#define SETOR1_DESLIGA      11          //Desliga alguma função - Setor 1
#define SETOR2_LIGA         20          //Liga alguma função - Setor 2
#define SETOR2_DESLIGA      21          //Desliga alguma função - Setor 2
#define TEMPO_INTERVALO     5           //Tempo de intervalo para iniciar o segundo setor

/* Variáveis globais */
bool flagManutencao = true;

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

        if (receive == DESLIGA_MANUTENCAO)
        {
            flagManutencao = false;
        }

        if (receive == LIGA_MANUTENCAO)
        {
            flagManutencao = true;
        }

        if (receive >= DESLIGA_RELES & receive <= LIGA_SETOR2 & flagManutencao == true)
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

void btnManutencaoPressionado()
{
    int result = 0;
    int comando = LIGA_MANUTENCAO;
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

void btnManutencaoliberado()
{
    int result = 0;
    int comando = DESLIGA_MANUTENCAO;
    bool erro = true;

    result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
    erro = dadoNaFila(result);
    if(!erro)
    {
        Serial.println("Erro ao colocar dado na fila");
        erro = true;
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
            vTaskDelay( 3000 / portTICK_PERIOD_MS ); 
            Reles.off(2);
            receive = 0;
            break;

        case 131:               //liga a irrigação do setor 1
            Reles.on(2);
            vTaskDelay( 3000 / portTICK_PERIOD_MS );
            Reles.on(0);
            receive = 0;
            break;

        case 140:               //desliga a irrigação do setor 2
            Reles.off(0);
            vTaskDelay( 3000 / portTICK_PERIOD_MS ); 
            Reles.off(3);
            receive = 0;
            break;

        case 141:               //liga a irrigação do setor 2
            Reles.on(3);
            vTaskDelay( 3000 / portTICK_PERIOD_MS );
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
   /* Estrutura de dados da struct tm                       */
   /*tm_sec;           segundo, faixa 0 to 59               */
   /*tm_min;           minutos, faixa 0 a 59                */
   /*tm_hora;          horas, intervalo de 0 a 23           */
   /*tm_mdia;          dia do mês, faixa 1 to 31            */
   /*tm_mon;           mês, intervalo de 0 a 11             */
   /*tm_ano;           O número de anos desde 1900          */
   /*tm_wday;          dia da semana, intervalo de 0 a 6    */
   /*tm_yday;          dia do ano, intervalo de 0 a 365     */
   /*tm_isdst;         horário de verão                     */

    int result = 0;
    int comando = 0;
    bool erro = true;
    int setorComando = 0;
    int tempoDecorrido = 0;
    bool iniciarContagem = false;
    byte enviaLigaSetor1 = 1;
    byte enviaDesligaSetor1 = 0;
    byte enviaLigaSetor2 = 0;
    byte enviaDesligaSetor2 = 0;

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
    
    /* Configuração tempo dos Setores */
    horarioSetor1.horaDeInicio = 19;
    horarioSetor1.minutoDeInicio = 10;
    horarioSetor1.tempoDeDuracao = 60;

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

            if(iniciarContagem == true)
            {
                tempoDecorrido++;

                Serial.print("Tempo decorrido: ");
                Serial.println(tempoDecorrido);
            }
            
            if(dias[diaDaSemana] == true)
            {
                if((horarioSetor1.horaDeInicio == timeinfo.tm_hour) & (horarioSetor1.minutoDeInicio == timeinfo.tm_min) & (enviaLigaSetor1 == 1))
                {
                    setorComando = SETOR1_LIGA;
                    iniciarContagem = true;
                    tempoDecorrido = 0;
                }
                
                if ((tempoDecorrido >= horarioSetor1.tempoDeDuracao) & (enviaDesligaSetor1 == 1))
                {
                    setorComando = SETOR1_DESLIGA;
                    iniciarContagem = false;
                }

                if((tempoDecorrido >= horarioSetor1.tempoDeDuracao) & (enviaLigaSetor2 == 1))
                {
                    vTaskDelay(5000 / portTICK_PERIOD_MS);     //5 segundos
                    setorComando = SETOR2_LIGA;
                    iniciarContagem = true;
                    tempoDecorrido = 0;
                    
                }
                if((tempoDecorrido >= horarioSetor1.tempoDeDuracao) & (enviaDesligaSetor2 == 1))
                {
                    setorComando = SETOR2_DESLIGA;
                    iniciarContagem = false;
                    tempoDecorrido = 0;
                }
            }
            else
            {
                Serial.println("Não precisa ligar");
            }
        }

        switch (setorComando)
        {
        case SETOR1_LIGA:
            comando = LIGA_SETOR1;

            result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
            erro = dadoNaFila(result);
            if(!erro)
            {
                Serial.println("Erro ao colocar dado na fila");
                erro = true;
            }
            comando = 0;
            setorComando = 0;
            enviaLigaSetor1 = 0;
            enviaDesligaSetor1 = 1;
            break;

        case SETOR1_DESLIGA:
            comando = DESLIGA_SETOR1;

            result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
            erro = dadoNaFila(result);
            if(!erro)
            {
                Serial.println("Erro ao colocar dado na fila");
                erro = true;
            }
            comando = 0;
            setorComando = 0;
            enviaDesligaSetor1 = 0;
            enviaLigaSetor2 = 1;
            break;

        case SETOR2_LIGA:
            comando = LIGA_SETOR2;

            result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
            erro = dadoNaFila(result);
            if(!erro)
            {
                Serial.println("Erro ao colocar dado na fila");
                erro = true;
            }
            comando = 0;
            setorComando = 0;
            enviaLigaSetor2 = 0;
            enviaDesligaSetor2 = 1;
            break;

        case SETOR2_DESLIGA:
            comando = DESLIGA_SETOR2;

            result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
            erro = dadoNaFila(result);
            if(!erro)
            {
                Serial.println("Erro ao colocar dado na fila");
                erro = true;
            }
            comando = 0;
            setorComando = 0;
            enviaDesligaSetor2 = 0;
            enviaLigaSetor1 = 1;

            //para teste, depois retirar
            horarioSetor1.minutoDeInicio += 6;
            if(horarioSetor1.minutoDeInicio > 59)
                {
                    horarioSetor1.minutoDeInicio = 0;
                    horarioSetor1.horaDeInicio++; 
                }
            break;
        
        default:
            comando = 0;
            setorComando = 0;
            break;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
