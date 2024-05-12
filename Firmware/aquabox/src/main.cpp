#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "SPIFFS.h"
#include "Sensores.hpp"

// Crie o objeto AsyncWebServer na porta 80
AsyncWebServer server(80);

// Procure parâmetro na solicitação HTTP POST
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ip";
const char* PARAM_INPUT_4 = "gateway";


//Variáveis para salvar valores do formulário HTML
String ssid;
String pass;
String ip;
String gateway;

// Caminhos de arquivo para salvar valores de entrada permanentemente
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ipPath = "/ip.txt";
const char* gatewayPath = "/gateway.txt";

IPAddress localIP;
//IPAddress localIP(192, 168, 1, 200); // Endereço local padrão

// Defina o endereço IP do seu gateway
IPAddress localGateway;
//IPAddress localGateway(192, 168, 1, 1); //Endereço Gateway padrão
IPAddress subnet(255, 255, 0, 0);

// Variáveis de temporizador
unsigned long previousMillis = 0;
const long interval = 10000;  // intervalo para esperar pela conexão Wi-Fi (milliseconds)

// Set LED GPIO
const int ledPin = 2;
// Armazena estado do LED

String ledState;

//Protótipo das funções
void initSPIFFS();
String readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
bool initWiFi();
String processor(const String& var);

void setup() 
{
  // Porta serial para fins de depuração
  Serial.begin(115200);

  //Inicializa o sistema de arquivos interno do ESP32
  initSPIFFS();

  // Defina GPIO 2 como uma SAÍDA
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  // Carregar valores salvos em SPIFFS
  ssid = readFile(SPIFFS, ssidPath);
  pass = readFile(SPIFFS, passPath);
  ip = readFile(SPIFFS, ipPath);
  gateway = readFile (SPIFFS, gatewayPath);
  Serial.println(ssid);
  Serial.println(pass);
  Serial.println(ip);
  Serial.println(gateway);

  if(initWiFi()) {
    // Rota para raiz/página da web
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });
    server.serveStatic("/", SPIFFS, "/");
    
    // Rota para definir o estado GPIO como HIGH
    server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request) {
      digitalWrite(ledPin, HIGH);
      request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });

    // Rota para definir o estado GPIO como LOW
    server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request) {
      digitalWrite(ledPin, LOW);
      request->send(SPIFFS, "/index.html", "text/html", false, processor);
    });
    server.begin();
  }
  else {
    // Conecte-se à rede Wi-Fi com SSID e senha
    Serial.println("configurando AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-WIFI-MANAGER", NULL);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP Endereço IP: ");
    Serial.println(IP); 

    // URL raiz do servidor Web
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });
    
    server.serveStatic("/", SPIFFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // Valor SSID HTTP POST
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            Serial.print("SSID definido como: ");
            Serial.println(ssid);
            // Gravar arquivo para salvar valor
            writeFile(SPIFFS, ssidPath, ssid.c_str());
          }
          // Valor de password HTTP POST
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass);
            // Gravar arquivo para salvar valor
            writeFile(SPIFFS, passPath, pass.c_str());
          }
          // Valor IP HTTP POST
          if (p->name() == PARAM_INPUT_3) {
            ip = p->value().c_str();
            Serial.print("IP Address set to: ");
            Serial.println(ip);
            // Gravar arquivo para salvar valor
            writeFile(SPIFFS, ipPath, ip.c_str());
          }
          // Valor do gateway HTTP POST
          if (p->name() == PARAM_INPUT_4) {
            gateway = p->value().c_str();
            Serial.print("Gateway set to: ");
            Serial.println(gateway);
            // Gravar arquivo para salvar valor
            writeFile(SPIFFS, gatewayPath, gateway.c_str());
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      request->send(200, "text/plain", "O ESP irá reiniciar, conectar-se ao seu roteador e acessar o endereço IP: " + ip);
      delay(3000);
      ESP.restart();
    });
    server.begin();
  }

  //Task de monitoramento e leitura dos sensores de nível
  xTaskCreate(taskSensores, "Sensores", 1024, NULL, 1, NULL);   

}

void loop() 
{
  
}

// Inicializa o sistema de arquivos SPIFFS
void initSPIFFS() 
{
  if (!SPIFFS.begin(true)) {
    Serial.println("Ocorreu um erro ao montar o SPIFFS");
  }
  Serial.println("SPIFFS montado com sucesso");
}

// Ler arquivo do SPIFFS
String readFile(fs::FS &fs, const char * path)
{
  Serial.printf("Lendo arquivo: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- não foi possível abrir o arquivo para leitura");
    return String();
  }
  
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;     
  }
  return fileContent;
}

// Gravar arquivo no SPIFFS
void writeFile(fs::FS &fs, const char * path, const char * message)
{
  Serial.printf("Escrevendo arquivo: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- não foi possível abrir o arquivo para gravação");
    return;
  }
  if(file.print(message)){
    Serial.println("- arquivo escrito");
  } else {
    Serial.println("- gravação falhou");
  }
}

// Inicializa o WiFi
bool initWiFi() 
{
  if(ssid=="" || ip==""){
    Serial.println("SSID ou endereço IP indefinido.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  localIP.fromString(ip.c_str());
  localGateway.fromString(gateway.c_str());


  if (!WiFi.config(localIP, localGateway, subnet)){
    Serial.println("Falha ao configurar STA");
    return false;
  }
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.println("Conectando-se ao Wi-Fi...");

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while(WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println("Falhou ao conectar.");
      return false;
    }
  }

  Serial.println(WiFi.localIP());
  return true;
}

// Substitui o espaço reservado pelo valor do estado do LED
String processor(const String& var) 
{
  if(var == "STATE") {
    if(digitalRead(ledPin)) {
      ledState = "ON";
    }
    else {
      ledState = "OFF";
    }
    return ledState;
  }
  return String();
}

