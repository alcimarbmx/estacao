#include <ESP8266WiFi.h>
#include <SimpleDHT.h>
#include <Adafruit_BMP085.h>
#include <OneWire.h>
/* A linha faz a importação da biblioteca do esp8266 
 * para que se possa fazer uso das funcões wifi deste modulo.
 * as linha 2 e 3 fazem a importação das bibliotecas dos sensoresdht11 
 * e bmp085 para que se possa fazer uso das funções pre-definidas pelos 
 * desenvolvedores destes modulos. */
int DS18S20_Pin = D4; //DS18S20 Signal pin on digital 2

//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2
Adafruit_BMP085 bmp;//cria a variavel que ira ser usada pelo bmp085.
//conf. roteador
const char* ssid     = "D-Link";//ssid da rede wi-fi, e senha, onde será conectado a placa nodemcu a web.
const char* password = "maria12345678";
/* Aqui é o endereço ip do servidor web, no qual será enviado o resultados das coletas 
 * para serem armazenadas em um banco de dados.*/
const char* host = "192.168.0.5";
/*sempre verificar qual o ip da maquina para que se possa enviar os dados
  para o endereco correto.*/

int luminosidade = 0, pinDHT11 = D3;
byte temperatura = 0, umidade = 0;
float bmpTemp, bmpPress, bmpAltt, bmpSeaP;

/* Cria um objeto da classe SimpleDHT11 que contem todas as funções para a 
   manipulação do sensor dht11.*/
SimpleDHT11 dht11;

void setup() {
  Serial.begin(9600);
  delay(10);
  /*iniciando a conexão com a rede wiriless*/
  Serial.println();
  Serial.println();
  Serial.print("Conectado à ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereço IP : ");
  Serial.println(WiFi.localIP());

/* verifica se o sensor bmp085 está devidamente conectado a placa, 
 * se ele não for identificado irá mostrar no monitor serial a linha 
 * seguinte dizendo que não foi possivel encontrá-lo. */
  if (!bmp.begin()) {
    Serial.println("Não foi possível encontrar o sensor bmp180!");
    while (1) {}
  }
}
void loop() {
  bmpTemp = bmp.readTemperature(); // temperatura
  bmpPress = bmp.readPressure(); // pressao atmosferica
  bmpAltt = bmp.readAltitude(); // altitude
  bmpSeaP = bmp.readSealevelPressure(); // pressao do nivel do mar

  //Sensor de temperatura do solo
  float tsolo = getTemp();

  //Sensor de luminosidade
  luminosidade = analogRead(A0);//faz a leitura do LDR e armazena na variavel, luminosidade.

  //sensor de temperatura e umidade relativa
  /*faz a leitura do dht11 e atribui o valor de cada informação as 
    variaveis precedidas do '&'. e se não for possivel fazer a leitura ele 
    caira na instrução seguinte, avisando que não foi possivel encontrar o sensor.*/
  if (dht11.read(pinDHT11, &temperatura, &umidade, NULL)) {
    Serial.print("Falha na leitura do DHT11.\n");return;}
 /*int err = SimpleDHTErrSuccess;
   if ((err = dht11.read(&temperatura, &umidade, NULL)) != SimpleDHTErrSuccess) {
     Serial.print("Read DHT11 failed, err="); Serial.println(err);//delay(1000);
    return;
  }*/
  Serial.print("conectado a ");//status da rede
  Serial.println(host);
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("Falha ao conectar com servidor web...");
    return;
  }
  // aqui é criada a url para enviar para o servidor 
  // We now create a URI for the request
  
 String url = "/nodemcu/salvar.php?luminosidade=";url += luminosidade;
  url += "&temperatura=";url += temperatura;
  url += "&umidade=";url += umidade;
  url += "&tsolo=";url += tsolo;
  url += "&bmpTemp=";url += bmpTemp;
  url += "&bmpPress=";url += bmpPress;
  url += "&bmpAltt=";url += bmpAltt;
  url += "&bmpSeaP=";url += bmpSeaP;
  /* A variavel url recebera todas as coletas em formato de url para em seguida 
  ser enviada via metodo get para uma pagina web que recebera esses dados. */
  Serial.print("Requesting URL: ");
  Serial.println(url);
  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");//envia a variavel url pelo metodo get diretamente para a pagina.
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }
  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    String line = client.readStringUntil('\r');

    if (line.indexOf("salvo_com_sucesso") != -1) {
          Serial.println("salvo com sucesso");
      }else if (line.indexOf("erro_ao_salvar") != -1) {
          Serial.println("erro ao salvar");
    }
  }
  Serial.println("fechando conexao");
  delay(10000);
}
//metodo para obtencao da leitura do sensor de temperatura do solo
float getTemp() {
  byte data[12];
  byte addr[8];
  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  ds.reset_search();
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}
