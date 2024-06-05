#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "WiFiManager.h"
#include "EventoSensores.hpp"
#include "Relogio.hpp"
#include "ModuloRele.hpp"
#include "EEPROM.h"
#include "DHT.h"


/* Libera prints para debug */
#define DEBUG

/* Pinos GPIOs */
#define BOMBA               36
#define SENSOR_DE_FLUXO     39
#define SENSOR_NIVEL_BAIXO  27
#define SENSOR_NIVEL_ALTO   14
#define RELE_BOMBA          2
#define RELE_CAIXA          15
#define RELE_SETOR_1        5
#define RELE_SETOR_2        4
#define BEEP                18 
#define UMIDADE             13

/* Comandos */
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
//#define LIGA_MANUTENCAO   201         //Liga manutenção
#define SETOR1_LIGA         10          //Liga alguma função - Setor 1
#define SETOR1_DESLIGA      11          //Desliga alguma função - Setor 1
#define SETOR2_LIGA         20          //Liga alguma função - Setor 2
#define SETOR2_DESLIGA      21          //Desliga alguma função - Setor 2
#define TEMPO_INTERVALO     5           //Tempo de intervalo para iniciar o segundo setor
#define CONFIGURACAO        300         //Comando para alterar a configuração de irrigação
#define STATUS              301         //Comando para o Aquabox enviar informações sobre o sistema
#define RE_START            302         //Reinicializar todo o sistema

/* Códigos de Retornos */
#define BOMBA_LIGADA            112         //Avisar que a bom esta ligada
#define BOMBA_DESLIGADA         113         //Avisar que a bomba esta desligada
#define CAIXA_VAZIA             122         //Informar que a caixa d'água esta vazia
#define CAIXA_CHEIA             123         //Informar que a caixa d'água esta cheia
#define CAIXA_NORMAL            124         //Informar que a caixa d'água esta normal e em uso
#define CAIXA_ENCHENDO          125         //Avisar que a caixa d'água está enchendo
#define SETOR1_LIGADO           132         //Avisar que o setor 1 da irrigação está ligado
#define SETOR1_DESLIGADO        133         //Avisar qye o setor 1 da irrigação está desligado
#define SETOR2_LIGADO           142         //Avisar que o setor 1 da irrigação está ligado
#define SETOR2_DESLIGADO        143         //Avisar qye o setor 1 da irrigação está desligado
#define MANUTENCAO_LIGADA       202         //Avisar que o botão de manutenção esta ligado (modo manutenção)
#define MANUTENCAO_DESLIGADA    203         //Avisar que o botão de manutenção esta desligado (fora do modo manutenção)
#define DIA_DE_CHUVA            204         //Informar que choveu e não vai ligar a irrigação

/* Códigos de erros e comandos aceitos */
#define SEM_ERROS               500         //Sistema funcionando normal
#define ERRO_DESCONHECIDO       501         //Erro não identificado
#define ERRO_DE_VAZAO           502         //Erro na vazão da bomba
#define ERRO_SENSOR_DE_VAZAO    503         //Erro no sensor de vazão. Se conseguir detectar o sensor de nível baixo
#define COMANDO_RECEBIDO        504         //Comando enviado via mqtt foi aceito e processado

/* Outras configurações */
#define TEMPO_BEEP_RAPIDO       200         //Tempo em milesegundos
#define INTERVALO_BEEPS         100         //Tempo em milesegundos
#define TEMPO_DE_ESPERA_VAZAO   30000       //Tempo em milesegundos (30 segundos)

/* Estrutura da EEPROM 
    Campo               Endereço
==================================    
modificado              (0)
horaDeInicio            (1)
minutoDeInicio          (2)
duracao                 (3)
diasDaSemana[0]         (4)     - dom
diasDaSemana[1]         (5)     - seg
diasDaSemana[2]         (6)     - ter
diasDaSemana[3]         (7)     - qua
diasDaSemana[4]         (8)     - qui
diasDaSemana[5]         (9)     - sex
diasDaSemana[6]         (10)    - sab
habilitaSensorVazao     (11)
habilitaSensorUmidade   (12)
*/

/* Demais defines */
#define TAMANHO_EEPROM 14
//#define MSG_BUFFER_SIZE 50
//char msg[MSG_BUFFER_SIZE];

/* Variáveis globais */
String msgRX;
//bool prontoParaEnviar = true;
bool prontoParaReceber = true;

// Variáveis para contagem de pulsos
volatile int contaPulso = 0;        //Variável para a quantidade de pulsos

const char* mqtt_server = "503847782e204ff99743e99127691fe7.s1.eu.hivemq.cloud";    //Host do broker
const int porta_TLS = 8883;                                                         //Porta
const char* mqtt_usuario = "Aquabox";                                               //Usuário
const char* mqtt_senha = "Liukin@0804";                                             //Senha do usuário
const char* topico_tx = "Aquabox/tx";                                               //Tópico para transmitir dados
const char* topico_rx = "Aquabox/rx";                                               //Tópico para receber dados

/* Variáveis globais *

/* Estruturas */
    struct irrigacaoConf
    {
        int modificado = 1;    //flag para indicar se estrutura foi alterada
        int horaDeInicio = 17;
        int minutoDeInicio = 0;
        int duracao = 20;  //20 minutos
        int diasDaSemana[7] = {1, 1, 1, 1, 1, 1, 1};
        int tempoDeDuracao = duracao*60;  //20 minutos      //Tempo utilizado no programa
        
    };

    struct statusAquabox
    {
        int statusBomba = BOMBA_DESLIGADA;
        int statusCaixa = CAIXA_NORMAL;
        int statusSetor1 = SETOR1_DESLIGADO;
        int statusSetor2 = SETOR2_DESLIGADO;
        int statusManutencao = MANUTENCAO_DESLIGADA;
        int statusErro = SEM_ERROS;
    };

    struct statusSensores
    {
        bool habilitaVazao = true;
        bool habilitaUmidade = true;
        bool erroDeVazao = false;
        float umidadeChuva = 95.00;
    };

    const char ALIAS1[] = "statusBomba";
    const char ALIAS2[] = "statusCaixa";
    const char ALIAS3[] = "statusSetor1";
    const char ALIAS4[] = "statusSetor2";
    const char ALIAS5[] = "statusManutencao";
    const char ALIAS6[] = "statusErro";
    const char ALIAS7[] = "statusDoComando";

    
struct irrigacaoConf conf_Irriga;
struct statusAquabox statusRetorno;
struct statusSensores habilitaSensor; 

/* Protótipo das funções e tasks */
void taskMqtt(void *params);
void taskControle(void *params);
void taskSensores(void *params);
void taskReles(void *params);
void taskRelogio(void *params);
void taskSensorDeFluxo(void *params);
void taskcalculaVazao(void *params);
void taskTrataErro(void *params);
void taskUmidadeTemperatura(void *params);
void btnBombaPressionado();
void btnBombaLiberado();
void nivelBaixoPressionado();
void nivelAltoPressionado();
void nivelBaixoLiberado();
void nivelAltoLiberado();
bool dadoNaFila(int result);
void escreverEEPROM();
void lerEEPROM();
void erroFila();
bool conecteMQTT();
void callbackMqtt(char *topico, byte *payload, unsigned int length);
void publicarMensagem(const char* topico, String payload);
void beepSinal(int duracao);     //Duracao em milesegundos
void sequenciaBeeps(int beeps, int duracao, int intervalo);  //Tempos em milisegundos
void leituraDePulsos();


/* filas (queues) */
QueueHandle_t xQueue_Reles, xQueue_Controle;

/* semaforos utilizados */
SemaphoreHandle_t xConfig_irrigacao, xStatusRetorno, xEnviaComando, xInterrupcaoVazao;

/* Objetos */
WiFiClientSecure espCliente;                //Estância o objeto cliente
PubSubClient cliente_MQTT(espCliente);      //Instancia o Cliente MQTT passando o objeto espClient
DHT dht(UMIDADE, DHT22);

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


void setup() 
{
    /* Inicializa serial (baudrate 115200) */
    Serial.begin(115200);

    /* Habilitar pino do BEEP */
    pinMode(BEEP, OUTPUT);

    /* Sinal sonoro */
    sequenciaBeeps(1, TEMPO_BEEP_RAPIDO, INTERVALO_BEEPS);

    /* Inicializa sensor dht22*/
    dht.begin();

    /* Inicialização da região de EEPROM */
    if(conf_Irriga.modificado == 1)
    {
        EEPROM.begin(TAMANHO_EEPROM);
        //escreverEEPROM();
        lerEEPROM();
    }
    
    /*Executa a conexão com WiFi via WiFiManager*/
    WiFi.mode(WIFI_STA);
    WiFiManager wm;
    bool res;
    res = wm.autoConnect("Aquabox");
    if(!res)
    {
        #ifdef DEBUG
            Serial.println("Falha ao conectar");
        #endif
    }
    else
    {
        #ifdef DEBUG
            Serial.println("Conectado...");
        #endif
    }
    
    //Criação da fila (Queue)
    ////Ao inicializar a fila devemos passar o tamanho dela e o tipo de dado. Pode ser inclusive estruturas
    xQueue_Reles = xQueueCreate(4, sizeof(int));
    xQueue_Controle = xQueueCreate(4, sizeof(int));

    /* Criação dos semaforos */
    xConfig_irrigacao = xSemaphoreCreateMutex();
    xStatusRetorno = xSemaphoreCreateMutex();
    xEnviaComando = xSemaphoreCreateMutex();
    xInterrupcaoVazao = xSemaphoreCreateMutex();

    //Task de monitoramento e leitura dos sensores de nível
    xTaskCreate(taskRelogio, "Relogio", 2048, NULL, 4, NULL);

    //Task para trabalhos co relógio de tempo real
    xTaskCreate(taskSensores, "Sensores", 2048, NULL, 5, NULL); 

    //Task de acionamento dos relés
    xTaskCreate(taskReles, "Reles", 2048, NULL, 5,NULL);

    //Task de controle
    xTaskCreate(taskControle, "Controle", 2048, NULL, 5, NULL); 

    //Task de comunicação MQTT
    xTaskCreate(taskMqtt, "mqtt", 4096, NULL, 5, NULL);     

    //Task contador de pulsos do sensor de fluxo
    xTaskCreate(taskSensorDeFluxo, "fluxo", 2048, NULL, 5, NULL);

    //Task para calcular vazão
    xTaskCreate(taskcalculaVazao, "vazao", 2048, NULL, 5, NULL);

    //Task para tratar os erros
    xTaskCreate(taskTrataErro, "erros", 2048, NULL, 4, NULL);

    //Task para o sensor de umidade e temperatura
    xTaskCreate(taskUmidadeTemperatura, "umidade", 2048, NULL, 4, NULL);
}

void loop() 
{
  
}

/* --------------------------------------------------*/
/* --------------- Tarefas / Funções ----------------*/
/* --------------------------------------------------*/

void taskMqtt(void *params)
{
    bool mqttStatus = 0;

    mqttStatus = conecteMQTT();

   while(true)
    {
        if(mqttStatus)
        {
            cliente_MQTT.loop();
        }
    }
}

bool conecteMQTT()
{
    byte tentativa = 0;
    espCliente.setCACert(root_ca);
    cliente_MQTT.setServer(mqtt_server, porta_TLS);
    cliente_MQTT.setCallback(callbackMqtt);

    do
    {
        /* Id do cliente MQTT - único */
        String cliente_id = "AQUABOX-";
        cliente_id += String(WiFi.macAddress());

        if(cliente_MQTT.connect(cliente_id.c_str(), mqtt_usuario, mqtt_senha))
    {
        #ifdef DEBUG
            Serial.println("Êxito na conexão:");
            Serial.printf("Cliente %s conectado ao broker\n", cliente_id.c_str());
        #endif
    }
    else
    {
        #ifdef DEBUG
            Serial.print("Falha ao conectar: ");
            Serial.print(cliente_MQTT.state());
            Serial.println();
            Serial.print("Tentativa: ");
            Serial.println(tentativa);
        #endif
        vTaskDelay( 2000 / portTICK_PERIOD_MS );
    }

    tentativa++;

    } while (!cliente_MQTT.connected() && tentativa < 5);

    if(tentativa < 5)
    {
        //publish e subscribe
        //cliente_MQTT.subscribe(topico_tx);
        cliente_MQTT.subscribe(topico_rx);
        //cliente_MQTT.publish(topico_tx, "{Teste de transmissão de MQTT}");
        return 1;
    }
    else
    {
        #ifdef DEBUG
            Serial.println("Não conectado");
        #endif
        return 0;
    }
    
}

void callbackMqtt(char *topico, byte *payload, unsigned int length)
{
    int result = 0;
    bool erro = true;
    int statusDoComando = COMANDO_RECEBIDO;

    /* Sinal sonoro */
    sequenciaBeeps(1, TEMPO_BEEP_RAPIDO, INTERVALO_BEEPS);

    #ifdef DEBUG
        //Serial.print("Mensagem recebida do tópico: ");
        //Serial.println(topico);
        //Serial.print("Mensagem: ");
    #endif
    for(int i = 0; i < length; i++)
    {
        //Serial.print((char) payload[i]);
        msgRX += (char)payload[i];
    }

    #ifdef DEBUG
    //Serial.println(msgRX);        //Retirar. apenas para teste
    #endif

    // Aloca o documento JSON
    // Entre colchetes, 200 é a capacidade do pool de memória em bytes.
    // Não se esqueça de alterar este valor para corresponder ao seu documento JSON.
    // Use arduinojson.org/v6/assistant para calcular a capacidade.
    //StaticJsonDocument<200> doc;
    JsonDocument doc;

    DeserializationError error = deserializeJson(doc, msgRX);

    if (error) {
        #ifdef DEBUG
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
        #endif
    return;
    }

    xSemaphoreTake(xEnviaComando, portMAX_DELAY);
    int comando = doc["comando"];
    xSemaphoreGive(xEnviaComando);

    if(comando >= DESLIGA_RELES && comando <= LIGA_SETOR2 )
    {
        /* Modelo do Json de configuração recebido */
        /*
            {
                "comando": "valor"
            }
        */

        result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
        erro = dadoNaFila(result);
        if(!erro)
        {
            erroFila();
            erro = true;
        }
        else
        {
            //Cria o objeto dinâmico "json" com tamanho "6" para a biblioteca
            JsonDocument json;
            //Atrela ao objeto "json" os dados definidos
            json[ALIAS7] = statusDoComando;
            //Mede o tamanho da mensagem "json" e atrela o valor somado em uma unidade ao objeto "tamanho_mensagem"
            size_t tamanho_payload = measureJson(json) + 1;

            //Cria a string "mensagem" de acordo com o tamanho do objeto "tamanho_mensagem"
            char payload[tamanho_payload];

            //Copia o objeto "json" para a variavel "payload" e com o "tamanho_payload"
            serializeJson(json, payload, tamanho_payload);

            //Publicar a variável "payload no servidor utilizando o tópico: topico/tx"
            publicarMensagem(topico_tx, payload);
        }
    }

    if(comando == CONFIGURACAO)
    {
        /* Modelo do Json de configuração - Exemplo */
        /*
            {
                "comando": "300",
                "horaDeInicio": "valor",
                "minutoDeInicio": "valor",
                "duracao": "valor",
                "dom": "1",
                "seg": "1",
                "ter": "1",
                "qua": "1",
                "qui": "1",
                "sex": "1",
                "sab": "1"
            }
        */

        xSemaphoreTake(xConfig_irrigacao, portMAX_DELAY);

        conf_Irriga.modificado = 0;
        conf_Irriga.horaDeInicio = doc["horaDeInicio"];
        conf_Irriga.minutoDeInicio = doc["minutoDeInicio"];
        conf_Irriga.duracao = doc["duracao"];
        conf_Irriga.diasDaSemana[0] = doc["dom"];
        conf_Irriga.diasDaSemana[1] = doc["seg"];
        conf_Irriga.diasDaSemana[2] = doc["ter"];
        conf_Irriga.diasDaSemana[3] = doc["qua"];
        conf_Irriga.diasDaSemana[4] = doc["qui"];
        conf_Irriga.diasDaSemana[5] = doc["sex"];
        conf_Irriga.diasDaSemana[6] = doc["sab"];
        conf_Irriga.tempoDeDuracao = conf_Irriga.duracao * 60;

        #ifdef DEBUG
        Serial.println(conf_Irriga.modificado);
        Serial.println(conf_Irriga.horaDeInicio);
        Serial.println(conf_Irriga.minutoDeInicio);
        Serial.println(conf_Irriga.duracao);
        Serial.println(conf_Irriga.diasDaSemana[0]);
        Serial.println(conf_Irriga.diasDaSemana[1]);
        Serial.println(conf_Irriga.diasDaSemana[2]);
        Serial.println(conf_Irriga.diasDaSemana[3]);
        Serial.println(conf_Irriga.diasDaSemana[4]);
        Serial.println(conf_Irriga.diasDaSemana[5]);
        Serial.println(conf_Irriga.diasDaSemana[6]);
        Serial.println(conf_Irriga.tempoDeDuracao);
        #endif

        /* Gravar na EEPROM */
        escreverEEPROM();
        /* Sinal sonoro */
        sequenciaBeeps(1, TEMPO_BEEP_RAPIDO, INTERVALO_BEEPS);
        lerEEPROM();
        comando = 0;
        xSemaphoreGive(xConfig_irrigacao);
    }

    if(comando == STATUS)
    {
        /* Modelo do Json de configuração recebido */
        /*
            {
                "comando": "301"
            }
        */

        result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
        erro = dadoNaFila(result);
        if(!erro)
        {
            erroFila();
            erro = true;
        }
        
        xSemaphoreTake(xEnviaComando, portMAX_DELAY);
        comando = 0;
        xSemaphoreGive(xEnviaComando);
    }

    if(comando == RE_START)
    {
        /* Modelo do Json de configuração recebido */
        /*
            {
                "comando": "302"
            }
        */
        /* Comando para reinicializar o ESP32 */
        esp_restart();
    }
    
    msgRX = "";
}

void publicarMensagem(const char* topico, String payload)
{
    if (cliente_MQTT.publish(topico, payload.c_str()))
    {
        #ifdef DEBUG
        Serial.println("Mensagem Publicada ["+String(topico)+"]: "+ payload);
        #endif
    }
}

void taskControle(void *params)
{
    /*Regra de negócio ficam aqui, centro de comando*/
    int receive = 0;
    int result = 0;
    bool erro = true;
    int checarPulsos = 0;

    while(true)
    {
        /* Espera até algo ser recebido na queue */
        xQueueReceive(xQueue_Controle, (void *)&receive, portMAX_DELAY);

        #ifdef DEBUG
            Serial.print("Comando chegou no controle: ");
            Serial.println (receive);
        #endif

        if ((receive >= DESLIGA_RELES) && (receive <= LIGA_SETOR2))
        {
            result = xQueueSend(xQueue_Reles, &receive, 500 / portTICK_PERIOD_MS);
            erro = dadoNaFila(result);
            if(!erro)
            {
                erroFila();
                erro = true;
            }
        }       

        if(receive == STATUS)
        {
            //Cria o objeto dinamico "json" com tamanho "6" para a biblioteca
            JsonDocument json;
            //Atrela ao objeto "json" os dados definidos
            json[ALIAS1] = statusRetorno.statusBomba;
            json[ALIAS2] = statusRetorno.statusCaixa;
            json[ALIAS3] = statusRetorno.statusSetor1;
            json[ALIAS4] = statusRetorno.statusSetor2;
            json[ALIAS5] = statusRetorno.statusManutencao;
            json[ALIAS6] = statusRetorno.statusErro;

            //Mede o tamanho da mensagem "json" e atrela o valor somado em uma unidade ao objeto "tamanho_mensagem"
            size_t tamanho_payload = measureJson(json) + 1;

            //Cria a string "mensagem" de acordo com o tamanho do objeto "tamanho_mensagem"
            char payload[tamanho_payload];

            //Copia o objeto "json" para a variavel "payload" e com o "tamanho_payload"
            serializeJson(json, payload, tamanho_payload);

            //Publicar a variável "payload no servidor utilizando o tópico: topico/tx"
            publicarMensagem(topico_tx, payload);

            #ifdef DEBUG
                Serial.print("Json enviado para topico: topico/tx ");
                Serial.println(payload);
            #endif
        }
    }
}

/* Task para tratamento dos essros */
void taskTrataErro(void *params)
{
    int desliga = 0;
    int result = 0;
    bool erro = true;
    int checarPulsos = 0;
    int repeticao = 0;
    
    while(true)
    {
        vTaskDelay( TEMPO_DE_ESPERA_VAZAO / portTICK_PERIOD_MS );          // Tempo de espera para checar se tem fluxo de água

         xSemaphoreTake(xEnviaComando, portMAX_DELAY);

        if((statusRetorno.statusBomba == BOMBA_LIGADA) && (habilitaSensor.habilitaVazao == true))
        {
            
            checarPulsos = contaPulso;
            vTaskDelay( 5000 / portTICK_PERIOD_MS );
            if(checarPulsos != contaPulso)
            {
                habilitaSensor.erroDeVazao = false;
                statusRetorno.statusErro = SEM_ERROS;
            }
            else
            {
                    repeticao++;
               
                if(repeticao >= 3)
                {
                    habilitaSensor.erroDeVazao = true;
                    statusRetorno.statusErro = ERRO_DE_VAZAO;
                    desliga = DESLIGA_RELES;
                    result = xQueueSend(xQueue_Controle, &desliga, 500 / portTICK_PERIOD_MS);
                    erro = dadoNaFila(result);
                    if(!erro)
                    {
                        erroFila();
                        erro = true;
                    }

                    repeticao = 0;
                }
                
                //TODO fazer rotina para enviar mensagem sobre erro de vazão
            }
        }

        xSemaphoreGive(xEnviaComando);
    }
}

void taskSensorDeFluxo(void *params)
{
    EventoSensores sensorDeFluxo(SENSOR_DE_FLUXO, LOW);

    sensorDeFluxo.setPressionadoCallback(&leituraDePulsos);

    while(true)
    {
        if(habilitaSensor.habilitaVazao == true)
        {
            sensorDeFluxo.process();
        }
        
        /*
        #ifdef DEBUG
        Serial.print("Contagem de pulsos: ");
        Serial.println(contaPulso);
        #endif
        */
    }
}

void leituraDePulsos()
{
    xSemaphoreTake(xInterrupcaoVazao, portMAX_DELAY);
    contaPulso++;
    xSemaphoreGive(xInterrupcaoVazao);
}

void taskcalculaVazao(void *params)
{
    double vazao;
    double tempVazao;
    //int temPulso = 0;
    int contadorDeTempo = 0;
    double tempoEmMinutos = 0.0;
    double litrosDeAgua = 0.0;

    while(true)
   {
    xSemaphoreTake(xInterrupcaoVazao, portMAX_DELAY);
    contaPulso = 0;
    xSemaphoreGive(xInterrupcaoVazao);

    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // Aguarde 1 segundo

    // Calcula a vazão em L/min
    xSemaphoreTake(xInterrupcaoVazao, portMAX_DELAY);
    //Trocar para 3.33mL
    vazao = contaPulso * 3.33 ;         //Conta os pulsos no último segundo e multiplica por 2,25mL, que é a vazão de cada pulso
    xSemaphoreGive(xInterrupcaoVazao);
    vazao = vazao * 60;                 //Converte segundos em minutos, tornando a unidade de medida mL/min
    vazao = vazao / 1000;               //Converte mL em litros, tornando a unidade de medida L/min
    
    //temPulso = temPulso + contaPulso;
    
    if(contaPulso > 0)
    {
        contadorDeTempo++;

        #ifdef DEBUG
            Serial.print("ContaPulsos: ");
            Serial.println(contaPulso);
            Serial.print("contadorDeTempo: ");
            Serial.println(contadorDeTempo);
        #endif
        tempVazao = vazao;
    }
    else
    {
        tempoEmMinutos = (double) contadorDeTempo/60;
        if(tempoEmMinutos != 0)
        {
            litrosDeAgua = tempoEmMinutos * tempVazao;
        }
        /*
        #ifdef DEBUG
            Serial.print("Vazão: ");
            Serial.print(tempVazao);
            Serial.println(" L/min");
            Serial.print("contadorDeTempo: ");
            Serial.print(contadorDeTempo);
            Serial.println(" seg ");
            Serial.print("tempoEmMInutos: ");
            Serial.print(tempoEmMinutos);
            Serial.println(" min ");
            Serial.println();
            Serial.print("Volume Total: ");
            Serial.print(litrosDeAgua);
            Serial.println(" litros");
        #endif
        */
        contadorDeTempo = 0;

    }
   litrosDeAgua = tempoEmMinutos * tempVazao;
   }
}

void taskUmidadeTemperatura(void *params)
{
    float temperatura;
    float umidade;

    vTaskDelay( 10000 / portTICK_PERIOD_MS ); // Aguarde 1 segundo

    while(true)
    {
        temperatura = dht.readTemperature();
        umidade = dht.readHumidity();

        #ifdef DEBUG
            Serial.println();
            Serial.print("Umidade: ");
            Serial.println(umidade);
            Serial.print("Temperatura: ");
            Serial.println(temperatura);
            Serial.println();

            if(umidade >= habilitaSensor.umidadeChuva)
            {
                Serial.println("Choveu!");
            }
        #endif

        vTaskDelay( 10000 / portTICK_PERIOD_MS ); // Aguarde 1 segundo
    }
}

void taskSensores(void *params)
{
    /* Estânciar objetos*/
    EventoSensores nivelBaixo(SENSOR_NIVEL_BAIXO, LOW);
    EventoSensores nivelAlto(SENSOR_NIVEL_ALTO, LOW);
    EventoSensores btnBomba(BOMBA, LOW);
    

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
    }
}

/*
void btnManutencaoPressionado()
{
    int result = 0;
    int comando = LIGA_MANUTENCAO;
    bool erro = true;

    #ifdef DEBUG
        Serial.println("Botão da Manutenção pressionado");
    #endif

    result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
    erro = dadoNaFila(result);
    if(!erro)
    {
        erroFila();
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
        erroFila();
        erro = true;
    }
}
*/
void btnBombaPressionado()
{
    int result = 0;
    int comando = LIGA_BOMBA;
    bool erro = true;

    #ifdef DEBUG
        Serial.println("Botão da Bomba pressionado");
    #endif

    result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
    erro = dadoNaFila(result);
    if(!erro)
    {
        erroFila();
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
        erroFila();
        erro = true;
    }
}

void nivelBaixoPressionado()
{
    int result = 0;
    int comando = LIGA_CAIXA;
    bool erro = true;
    xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
    statusRetorno.statusCaixa = CAIXA_VAZIA;
    xSemaphoreGive(xStatusRetorno);

    result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
    erro = dadoNaFila(result);
    if(!erro)
    {
        erroFila();
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
        erroFila();
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
    xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
    statusRetorno.statusCaixa = CAIXA_NORMAL;
    xSemaphoreGive(xStatusRetorno);
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

        #ifdef DEBUG
            Serial.print("Comando chegou nos relés: ");
            Serial.println (receive);
        #endif

        switch (receive)
        {
        case 100:               //Desliga todos os relés
            xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
            statusRetorno.statusBomba = BOMBA_DESLIGADA;
            prontoParaReceber = true;
            xSemaphoreGive(xStatusRetorno); 
            Reles.offAll();
            receive = 0;
            break;

        case 110:               //desliga somente a bomba
            xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
            statusRetorno.statusBomba = BOMBA_DESLIGADA;
            prontoParaReceber = true;
            xSemaphoreGive(xStatusRetorno);            
            Reles.off(0);
            receive = 0;
            prontoParaReceber = true;
            break;

        case 111:               //Liga somente a bomba
            xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
            statusRetorno.statusBomba = BOMBA_LIGADA;
            prontoParaReceber = false;
            xSemaphoreGive(xStatusRetorno);                
            Reles.on(0);
            receive = 0;
            beepSinal(TEMPO_BEEP_RAPIDO);
            break;

        case 120:               //desliga o encher da caixa d'água
            xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
            statusRetorno.statusCaixa = CAIXA_CHEIA;
            statusRetorno.statusBomba = BOMBA_DESLIGADA;
            prontoParaReceber = true;
            xSemaphoreGive(xStatusRetorno);                                            
            Reles.off(0);
            vTaskDelay( 3000 / portTICK_PERIOD_MS ); 
            Reles.off(1);
            receive = 0;
            break;

        case 121:               //liga o encher da caixa d'água
            xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
            statusRetorno.statusCaixa = CAIXA_ENCHENDO;
            statusRetorno.statusBomba = BOMBA_LIGADA;
            prontoParaReceber = false;
            xSemaphoreGive(xStatusRetorno);                               
            Reles.on(1);
            vTaskDelay( 3000 / portTICK_PERIOD_MS );
            Reles.on(0);
            receive = 0;
            break;

        case 130:               //desliga a irrigação do setor 1
            xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
            statusRetorno.statusSetor1 = SETOR1_DESLIGADO;
            statusRetorno.statusBomba = BOMBA_DESLIGADA;
            prontoParaReceber = true;
            xSemaphoreGive(xStatusRetorno);                              
            Reles.off(0);
            vTaskDelay( 3000 / portTICK_PERIOD_MS ); 
            Reles.off(2);
            receive = 0;
            break;

        case 131:               //liga a irrigação do setor 1
            xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
            statusRetorno.statusSetor1 = SETOR1_LIGADO;
            statusRetorno.statusBomba = BOMBA_LIGADA;
            prontoParaReceber = false;
            xSemaphoreGive(xStatusRetorno);
            Reles.on(2);
            vTaskDelay( 3000 / portTICK_PERIOD_MS );
            Reles.on(0);
            receive = 0;
            break;

        case 140:               //desliga a irrigação do setor 2
            xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
            statusRetorno.statusSetor2 = SETOR2_DESLIGADO;
            statusRetorno.statusBomba = BOMBA_DESLIGADA;
            prontoParaReceber = true;
            xSemaphoreGive(xStatusRetorno);
            Reles.off(0);
            vTaskDelay( 3000 / portTICK_PERIOD_MS ); 
            Reles.off(3);
            receive = 0;
            break;

        case 141:               //liga a irrigação do setor 2
            xSemaphoreTake(xStatusRetorno, portMAX_DELAY);
            statusRetorno.statusSetor2 = SETOR2_LIGADO;
            statusRetorno.statusBomba = BOMBA_LIGADA;
            prontoParaReceber = false;
            xSemaphoreGive(xStatusRetorno);
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

void taskRelogio(void *params)
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
    byte enviaLigaSetor1 = 0;
    byte enviaDesligaSetor1 = 0;
    byte enviaLigaSetor2 = 0;
    byte enviaDesligaSetor2 = 0;
    byte naoRepeteComando = 1;

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
            #ifdef DEBUG
                Serial.println("Falha ao obter o relógio");
            #endif
        }
        else
        {
            diaDaSemana = timeinfo.tm_wday;

            if(iniciarContagem == true)
            {
                tempoDecorrido++;
            }
            
            xSemaphoreTake(xEnviaComando, portMAX_DELAY);

            if(conf_Irriga.diasDaSemana[diaDaSemana] == 1)
            {
                if((conf_Irriga.horaDeInicio == timeinfo.tm_hour) && (conf_Irriga.minutoDeInicio == timeinfo.tm_min) && (naoRepeteComando == 1))
                {
                    enviaLigaSetor1 = 1;
                    naoRepeteComando = 0;
                }

                if((enviaLigaSetor1 == 1) && (prontoParaReceber == true))
                {
                    enviaLigaSetor1 = 0;
                    setorComando = SETOR1_LIGA;
                    iniciarContagem = true;
                    tempoDecorrido = 0;
                }

                if ((tempoDecorrido >= conf_Irriga.tempoDeDuracao) && (enviaDesligaSetor1 == 1))
                {
                    setorComando = SETOR1_DESLIGA;
                    iniciarContagem = false;
                }

                if((tempoDecorrido >= conf_Irriga.tempoDeDuracao) && (enviaLigaSetor2 == 1))
                {
                    vTaskDelay(5000 / portTICK_PERIOD_MS);     //5 segundos
                    setorComando = SETOR2_LIGA;
                    iniciarContagem = true;
                    tempoDecorrido = 0;
                    
                }
                if((tempoDecorrido >= conf_Irriga.tempoDeDuracao) && (enviaDesligaSetor2 == 1))
                {
                    setorComando = SETOR2_DESLIGA;
                    iniciarContagem = false;
                    tempoDecorrido = 0;
                }
            }
            else
            {
                #ifdef DEBUG
                    Serial.println("Não precisa ligar");
                #endif
            }

            xSemaphoreGive(xEnviaComando);
        }

        switch (setorComando)
        {
        case SETOR1_LIGA:
            comando = LIGA_SETOR1;

            result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
            erro = dadoNaFila(result);
            if(!erro)
            {
                erroFila();
                erro = true;
            }

            xSemaphoreTake(xEnviaComando, portMAX_DELAY);
            comando = 0;
            setorComando = 0;
            enviaLigaSetor1 = 0;
            enviaDesligaSetor1 = 1;
            xSemaphoreGive(xEnviaComando);
            break;

        case SETOR1_DESLIGA:
            comando = DESLIGA_SETOR1;

            result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
            erro = dadoNaFila(result);
            if(!erro)
            {
                erroFila();
                erro = true;
            }

            xSemaphoreTake(xEnviaComando, portMAX_DELAY);
            comando = 0;
            setorComando = 0;
            enviaDesligaSetor1 = 0;
            enviaLigaSetor2 = 1;
            xSemaphoreGive(xEnviaComando);
            break;

        case SETOR2_LIGA:
            comando = LIGA_SETOR2;

            result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
            erro = dadoNaFila(result);
            if(!erro)
            {
                erroFila();
                erro = true;
            }

            xSemaphoreTake(xEnviaComando, portMAX_DELAY);
            comando = 0;
            setorComando = 0;
            enviaLigaSetor2 = 0;
            enviaDesligaSetor2 = 1;
            xSemaphoreGive(xEnviaComando);
            break;

        case SETOR2_DESLIGA:
            comando = DESLIGA_SETOR2;

            result = xQueueSend(xQueue_Controle, &comando, 500 / portTICK_PERIOD_MS);
            erro = dadoNaFila(result);
            if(!erro)
            {
                erroFila();
                erro = true;
            }

            xSemaphoreTake(xEnviaComando, portMAX_DELAY);
            comando = 0;
            setorComando = 0;
            enviaDesligaSetor2 = 0;
            enviaLigaSetor1 = 0;
            naoRepeteComando = 1;
            xSemaphoreGive(xEnviaComando);
            break;
        
        default:
            xSemaphoreTake(xEnviaComando, portMAX_DELAY);
            comando = 0;
            setorComando = 0;
            xSemaphoreGive(xEnviaComando);
            break;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void escreverEEPROM()
{
    EEPROM.write(0,conf_Irriga.modificado);
    EEPROM.write(1,conf_Irriga.horaDeInicio);
    EEPROM.write(2,conf_Irriga.minutoDeInicio);
    EEPROM.write(3,(conf_Irriga.duracao));
    EEPROM.write(4,conf_Irriga.diasDaSemana[0]);
    EEPROM.write(5,conf_Irriga.diasDaSemana[1]);
    EEPROM.write(6,conf_Irriga.diasDaSemana[2]);
    EEPROM.write(7,conf_Irriga.diasDaSemana[3]);
    EEPROM.write(8,conf_Irriga.diasDaSemana[4]);
    EEPROM.write(9,conf_Irriga.diasDaSemana[5]);
    EEPROM.write(10,conf_Irriga.diasDaSemana[6]);
    EEPROM.write(11,habilitaSensor.habilitaUmidade);
    EEPROM.write(12,habilitaSensor.habilitaVazao);

    EEPROM.commit();

    #ifdef DEBUG
        Serial.println("EEPROM Gravada");
    #endif

}

void lerEEPROM()
{
    conf_Irriga.modificado = EEPROM.read(0);
    conf_Irriga.horaDeInicio = EEPROM.read(1);
    conf_Irriga.minutoDeInicio = EEPROM.read(2);
    conf_Irriga.duracao = EEPROM.read(3);
    conf_Irriga.diasDaSemana[0] = EEPROM.read(4);
    conf_Irriga.diasDaSemana[1] = EEPROM.read(5);
    conf_Irriga.diasDaSemana[2] = EEPROM.read(6);
    conf_Irriga.diasDaSemana[3] = EEPROM.read(7);
    conf_Irriga.diasDaSemana[4] = EEPROM.read(8);
    conf_Irriga.diasDaSemana[5] = EEPROM.read(9);
    conf_Irriga.diasDaSemana[6] = EEPROM.read(10);
    habilitaSensor.habilitaUmidade = EEPROM.read(11);
    habilitaSensor.habilitaVazao = EEPROM.read(12);

    #ifdef DEBUG
        Serial.println("EEPROM lida");
        Serial.println(conf_Irriga.modificado);
        Serial.println(conf_Irriga.horaDeInicio);
        Serial.println(conf_Irriga.minutoDeInicio);
        Serial.println(conf_Irriga.duracao);
        Serial.println(conf_Irriga.diasDaSemana[0]);
        Serial.println(conf_Irriga.diasDaSemana[1]);
        Serial.println(conf_Irriga.diasDaSemana[2]);
        Serial.println(conf_Irriga.diasDaSemana[3]);
        Serial.println(conf_Irriga.diasDaSemana[4]);
        Serial.println(conf_Irriga.diasDaSemana[5]);
        Serial.println(conf_Irriga.diasDaSemana[6]);
        Serial.println(conf_Irriga.tempoDeDuracao);
        Serial.println(habilitaSensor.habilitaUmidade);
        Serial.println(habilitaSensor.habilitaVazao);
    #endif
}

void erroFila()
{
    #ifdef DEBUG
        Serial.println("Erro ao colocar dado na fila");
    #endif
}

void beepSinal(int duracao)     //Duracao em milesegundos
{
    digitalWrite(BEEP, HIGH);
    vTaskDelay( duracao / portTICK_PERIOD_MS );
    digitalWrite(BEEP, LOW);
}

void sequenciaBeeps(int beeps, int duracao, int intervalo)
{
    for(int i = 0; i < beeps; i++)
    {
        beepSinal(duracao);
        vTaskDelay( intervalo / portTICK_PERIOD_MS );
    }
}
