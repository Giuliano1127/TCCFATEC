#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ======================================================
// WI-FI E MQTT — PREENCHA COM SEUS DADOS
// ======================================================

const char* SSID = "";
const char* SENHA = "";
const char* BROKER = "";
const int PORTA_MQTT = ;

// ======================================================
// TÓPICOS MQTT
// ======================================================

const char* TOPICO_COMANDO = "COMANDO";
const char* TOPICO_ZERAR = "zerar";
const char* TOPICO_CORRENTE = "Corrente";
const char* TOPICO_ALTURA = "Altura";
const char* TOPICO_TEMPERATURA = "Temperatura";
const char* TOPICO_STATUS = "Status";
const char* TOPICO_CONEXAO = "Conexao";   // "online" / "offline"

// ======================================================
// PINOS — ESP32
// ======================================================

const int PIN_ACS = 34;
const int PIN_DS18B20 = 4;
const int PIN_ENCODER = 32;

const int PIN_RPWM = 25;
const int PIN_LPWM = 26;
const int PIN_EN = 27;

// ======================================================
// ACS712
// ======================================================

// ACS712 5 A: 0.185 V/A
// ACS712 20 A: 0.100 V/A
// ACS712 30 A: 0.066 V/A
const float SENSIBILIDADE_ACS = 0.185;

// Sem divisor entre ACS OUT e GPIO34, deixe 1.0.
// Com divisor, ajuste pelo divisor real.
// Exemplo: se a tensão no GPIO for metade da saída do ACS, use 2.0.
const float FATOR_DIVISOR_ACS = 1.0;

// ======================================================
// TAMBOR, ENCODER E CURSO
// ======================================================

const float DIAMETRO_TAMBOR_MM = 14.0;
const int FUROS_ENCODER = 20;
const float CURSO_MAXIMO_MM = 500.0;

const float MM_POR_PULSO =
  (PI * DIAMETRO_TAMBOR_MM) / FUROS_ENCODER;

// Para os valores acima: aproximadamente 2.20 mm por pulso.

// ======================================================
// INTERVALOS
// ======================================================

const unsigned long INTERVALO_MQTT_MS = 500;
const unsigned long INTERVALO_TEMP_MS = 1000;
const unsigned long INTERVALO_WIFI_MS = 10000;
const unsigned long INTERVALO_RECONEXAO_MQTT_MS = 5000;
const unsigned long TEMPO_CONVERSAO_DS18B20_MS = 200;

// ======================================================
// OBJETOS
// ======================================================

WiFiClient espClient;
PubSubClient mqtt(espClient);

OneWire oneWire(PIN_DS18B20);
DallasTemperature sensorTemp(&oneWire);

// ======================================================
// VARIÁVEIS
// ======================================================

volatile uint32_t pulsosEncoder = 0;
uint32_t ultimoPulsosEncoder = 0;

float offsetACS_mV = 2500.0;
float correnteA = 0.0;
float temperaturaC = 0.0;
float deslocamentoMM = 0.0;

bool temperaturaValida = false;
bool conversaoTemperaturaPendente = false;

int direcaoMotor = 0; // 1 = sobe, -1 = desce, 0 = parado

unsigned long tempoMQTT = 0;
unsigned long tempoWiFi = 0;
unsigned long tempoReconexaoMQTT = 0;
unsigned long ultimaSolicitacaoTemperatura = 0;
unsigned long inicioConversaoTemperatura = 0;

String statusEvento = "parado";

// ======================================================
// ENCODER
// ======================================================

// Esta função é chamada a cada pulso do encoder.
void IRAM_ATTR contarPulsoEncoder() {
  pulsosEncoder++;
}

uint32_t lerPulsosEncoder() {
  noInterrupts();
  uint32_t pulsos = pulsosEncoder;
  interrupts();

  return pulsos;
}

void zerarEncoderHardware() {
  noInterrupts();
  pulsosEncoder = 0;
  interrupts();

  ultimoPulsosEncoder = 0;
}

void atualizarDeslocamento() {
  uint32_t pulsosAtuais = lerPulsosEncoder();
  uint32_t delta = pulsosAtuais - ultimoPulsosEncoder;

  ultimoPulsosEncoder = pulsosAtuais;

  // Conta somente durante a subida.
  if (direcaoMotor == 1 && delta > 0) {
    deslocamentoMM += (float)delta * MM_POR_PULSO;

    if (deslocamentoMM >= CURSO_MAXIMO_MM) {
      deslocamentoMM = CURSO_MAXIMO_MM;
      acionarMotor(0, "limite_superior");
    }
  }
}

// ======================================================
// MOTOR — BTS7960
// ======================================================

void aplicarPWM(int pwmVal, int direcao) {
  pwmVal = constrain(pwmVal, 0, 255);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_RPWM, direcao == 1 ? pwmVal : 0);
  ledcWrite(PIN_LPWM, direcao == -1 ? pwmVal : 0);
#else
  ledcWrite(0, direcao == 1 ? pwmVal : 0);
  ledcWrite(1, direcao == -1 ? pwmVal : 0);
#endif
}

void acionarMotor(int direcao, const char* status) {
  direcaoMotor = direcao;
  statusEvento = status;

  if (direcao != 0) {
    digitalWrite(PIN_EN, HIGH);
    aplicarPWM(255, direcao);
  } else {
    aplicarPWM(0, 0);
    digitalWrite(PIN_EN, LOW);
  }
}

// ======================================================
// SEGURANÇA: DESLIGA O MOTOR SE A CONEXÃO CAIR
// ======================================================

void verificarConexao() {
  if (direcaoMotor != 0 &&
      (WiFi.status() != WL_CONNECTED || !mqtt.connected())) {
    acionarMotor(0, "sem_conexao");
    Serial.println("Conexao perdida: motor desligado.");
  }
}

// ======================================================
// ACS712 — CORRENTE
// ======================================================

void calibrarACS() {
  Serial.println("Calibrando ACS712: mantenha o motor desligado.");

  const int AMOSTRAS = 500;
  long soma_mV = 0;

  for (int i = 0; i < AMOSTRAS; i++) {
    soma_mV += analogReadMilliVolts(PIN_ACS);
    delay(2);
  }

  float mediaNoGPIO_mV = (float)soma_mV / AMOSTRAS;
  offsetACS_mV = mediaNoGPIO_mV * FATOR_DIVISOR_ACS;

  Serial.print("Offset ACS: ");
  Serial.print(offsetACS_mV, 1);
  Serial.println(" mV");
}

float lerCorrente() {
  const int AMOSTRAS = 100;
  long soma_mV = 0;

  for (int i = 0; i < AMOSTRAS; i++) {
    soma_mV += analogReadMilliVolts(PIN_ACS);
    delayMicroseconds(200);
  }

  float mediaNoGPIO_mV = (float)soma_mV / AMOSTRAS;
  float tensaoACS_mV = mediaNoGPIO_mV * FATOR_DIVISOR_ACS;
  float diferenca_mV = tensaoACS_mV - offsetACS_mV;

  float corrente = diferenca_mV / (SENSIBILIDADE_ACS * 1000.0);

  if (fabs(corrente) < 0.05) {
    return 0.0;
  }

  return fabs(corrente);
}

// ======================================================
// DS18B20 — TEMPERATURA
// ======================================================

void atualizarTemperatura() {
  unsigned long agora = millis();

  if (
    conversaoTemperaturaPendente &&
    agora - inicioConversaoTemperatura >= TEMPO_CONVERSAO_DS18B20_MS
  ) {
    float leitura = sensorTemp.getTempCByIndex(0);

    if (
      leitura != DEVICE_DISCONNECTED_C &&
      leitura >= -55.0 &&
      leitura <= 125.0
    ) {
      temperaturaC = leitura;
      temperaturaValida = true;
    } else {
      temperaturaValida = false;
    }

    conversaoTemperaturaPendente = false;
  }

  if (
    !conversaoTemperaturaPendente &&
    agora - ultimaSolicitacaoTemperatura >= INTERVALO_TEMP_MS
  ) {
    sensorTemp.requestTemperatures();
    conversaoTemperaturaPendente = true;
    inicioConversaoTemperatura = agora;
    ultimaSolicitacaoTemperatura = agora;
  }
}

// ======================================================
// COMANDOS MQTT
// ======================================================

void processarComando(String mensagem) {
  mensagem.trim();
  mensagem.toLowerCase();

  Serial.print("Comando recebido: ");
  Serial.println(mensagem);

  if (mensagem == "zerar") {
    acionarMotor(0, "deslocamento_zerado");
    zerarEncoderHardware();
    deslocamentoMM = 0.0;
  }
  else if (
    mensagem == "subir" ||
    mensagem == "1" ||
    mensagem == "true"
  ) {
    if (deslocamentoMM < CURSO_MAXIMO_MM) {
      acionarMotor(1, "subindo");
    } else {
      acionarMotor(0, "limite_superior");
    }
  }
  else if (
    mensagem == "descer" ||
    mensagem == "-1"
  ) {
    // A descida funciona mesmo que a altura medida esteja em zero.
    // A altura não é subtraída durante a descida.
    acionarMotor(-1, "descendo");
  }
  else if (
    mensagem == "parar" ||
    mensagem == "0" ||
    mensagem == "false"
  ) {
    acionarMotor(0, "parado");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) == TOPICO_ZERAR) {
    processarComando("zerar");
    return;
  }

  String mensagem;
  mensagem.reserve(length + 1);

  for (unsigned int i = 0; i < length; i++) {
    mensagem += (char)payload[i];
  }

  processarComando(mensagem);
}

// ======================================================
// WI-FI E MQTT
// ======================================================

void conectarWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  unsigned long agora = millis();

  if (agora - tempoWiFi < INTERVALO_WIFI_MS) {
    return;
  }

  tempoWiFi = agora;
  Serial.println("Conectando ao Wi-Fi...");
  WiFi.disconnect();
  WiFi.begin(SSID, SENHA);
}

void conectarMQTT() {
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED) {
    return;
  }

  String clientID =
    "ESP32_TCC_" + String((uint32_t)ESP.getEfuseMac(), HEX);

  Serial.println("Conectando ao MQTT...");

  // Last Will: se o ESP32 sumir da rede, o broker publica "offline"
  if (mqtt.connect(
        clientID.c_str(),
        NULL,
        NULL,
        TOPICO_CONEXAO,
        1,
        true,
        "offline"
      )) {
    Serial.println("MQTT conectado.");
    mqtt.subscribe(TOPICO_COMANDO);
    mqtt.subscribe(TOPICO_ZERAR);
    mqtt.publish(TOPICO_CONEXAO, "online", true);
  } else {
    Serial.print("Falha MQTT. Estado: ");
    Serial.println(mqtt.state());
  }
}

// ======================================================
// PUBLICAÇÃO MQTT
// ======================================================

void publicarDados() {
  if (!mqtt.connected()) {
    return;
  }

  correnteA = lerCorrente();

  mqtt.publish(TOPICO_CORRENTE, String(correnteA, 2).c_str());

  // Altura publicada em centímetros.
  mqtt.publish(TOPICO_ALTURA, String(deslocamentoMM / 10.0, 1).c_str());

  if (temperaturaValida) {
    mqtt.publish(
      TOPICO_TEMPERATURA,
      String(temperaturaC, 1).c_str()
    );
  }

  String statusAtual = statusEvento;

  if (!temperaturaValida) {
    statusAtual = "verificar_temperatura";
  }

  mqtt.publish(TOPICO_STATUS, statusAtual.c_str());
}

// ======================================================
// SETUP
// ======================================================

void setup() {
  Serial.begin(115200);

  pinMode(PIN_ACS, INPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);

  // Encoder com pull-up. Pressupõe saída que puxa o sinal para GND.
  pinMode(PIN_ENCODER, INPUT_PULLUP);

  // Conteúdo de pulso na borda de descida.
  attachInterrupt(
    digitalPinToInterrupt(PIN_ENCODER),
    contarPulsoEncoder,
    FALLING
  );

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_RPWM, 5000, 8);
  ledcAttach(PIN_LPWM, 5000, 8);
#else
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(PIN_RPWM, 0);
  ledcAttachPin(PIN_LPWM, 1);
#endif

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ACS, ADC_11db);

  sensorTemp.begin();
  sensorTemp.setResolution(10);
  sensorTemp.setWaitForConversion(false);

  calibrarACS();

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, SENHA);

  mqtt.setServer(BROKER, PORTA_MQTT);
  mqtt.setCallback(callback);
  mqtt.setKeepAlive(5);      // detecta queda do broker mais rápido
  mqtt.setSocketTimeout(2);  // evita travar o loop ao reconectar

  tempoWiFi = millis() - INTERVALO_WIFI_MS;
  tempoReconexaoMQTT = millis() - INTERVALO_RECONEXAO_MQTT_MS;
  tempoMQTT = millis();
  ultimaSolicitacaoTemperatura = millis() - INTERVALO_TEMP_MS;

  Serial.println("Sistema pronto.");
  Serial.println("Posicione o cabo na referência e envie zerar.");
}

// ======================================================
// LOOP
// ======================================================

void loop() {
  unsigned long agora = millis();

  // A segurança vem antes de qualquer tentativa de reconexão
  verificarConexao();

  conectarWiFi();

  if (
    WiFi.status() == WL_CONNECTED &&
    !mqtt.connected() &&
    agora - tempoReconexaoMQTT >= INTERVALO_RECONEXAO_MQTT_MS
  ) {
    tempoReconexaoMQTT = agora;
    conectarMQTT();
  }

  if (mqtt.connected()) {
    mqtt.loop();
  }

  atualizarDeslocamento();
  atualizarTemperatura();

  if (agora - tempoMQTT >= INTERVALO_MQTT_MS) {
    tempoMQTT = agora;

    publicarDados();

    uint32_t pulsos = lerPulsosEncoder();

    Serial.printf(
      "Corrente: %.2f A | Temperatura: %.1f C | "
      "Altura: %.1f cm | Pulsos totais: %lu | Motor: %d\n",
      correnteA,
      temperaturaC,
      deslocamentoMM / 10.0,
      (unsigned long)pulsos,
      direcaoMotor
    );
  }
}
