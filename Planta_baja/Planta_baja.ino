// BOF preprocessor bug prevent - insert me on top of your arduino-code
// From: http://www.a-control.de/arduino-fehler/?lang=en
#if 1
__asm volatile ("nop");
#endif
#include   "Excontrol_def.h"
//Enable watch dogf
//habilita perro guardian
#define EXC_ENABLE_WATCH_DOG
#define EXC_INTERNAL_RESISTOR
//#define EXC_WIFI_SHIELD
//#define EXC_I2C_BOARD 1
//#define ENABLE_SMODBUS


/********NRF24L01******************************************************************************/

//#define EXC_NRF24
//#define EXC_NRF24SPEED 1
//#define EXC_DEBUB_NRF24

/********NRF24L01******************************************************************************/

#define EXC_ARDUINO_MEGA
#define EXC_DEBUG_MODE

//#define EXC_SERVER

/***************************SETTING EXTERNAL CONECTION**************************************/
//#define ExControlMail "example@use.com"
#define ExControlPass "******"

//****************************************************
//CONFIGURACION EQUIPOS INTALADOS, TERMOSTATOS, ENCHUFES RADIOFRECUENCIA 433MHZ, INFARROJOS.
//EQUIPMENT CONFIGURATION , THERMOSTAT, RADIO 433MHZ, INFARROJOS.



#define EXC_LCD
//#define EXC_LED_IR
//#define EXC_IR_RECIVE
//#define EXC_RECEIVER_433
//#define EXC_TRANSMITER_433
//#define EXC_RGB_LED
#define THERMOSTAT_DS18B20_NUMBER 3
//#define THERMOSTAT_DTH22
//#define EXC_NumeroPersianas 1

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/



#ifdef EXC_ARDUINO_MEGA
  #define SD_CARD
#endif
#ifdef ENABLE_SMODBUS
//#define MODBUS_BAUD 19200
#endif

#ifdef EXC_ENABLE_WATCH_DOG
#include <avr/wdt.h>
#endif 
#include <SPI.h>  
#ifdef EXC_WIFI_SHIELD
//#include <WiFi.h>
//#include <WiFiUdp.h>
#else
#include <Ethernet.h>
#include <EthernetUdp.h>
#endif
#ifdef EXC_ARDUINO_MEGA
  #ifdef SD_CARD
#include <SD.h>
  #endif
#include <EEPROM.h>
#else

  //Clock addres
  //DS3231= 57
  //ds1307=50
  #define IC24C32_I2C_ADDRESS 0x57
#endif 

#include <Wire.h>
#define UDP_TX_PACKET_MAX_SIZE 100 //increase UDP size

#if defined (EXC_TRANSMITER_433)  || defined (EXC_RECEIVER_433)
//#include <RCSwitch.h>
#endif

#if defined (EXC_LED_IR)  || defined (EXC_IR_RECIVE)
//#include <IRremote.h>
#endif



#ifdef ENABLE_SMODBUS
  #include <SimpleModbusSlave.h>
#endif
#ifdef THERMOSTAT_DS18B20_NUMBER 
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif 
#ifdef EXC_LCD 
 #include <LiquidCrystal_I2C.h>
 #include   "printLCD.h"
#endif 

#ifdef THERMOSTAT_DTH22 
  //#include "DHT.h"

#endif 

#ifdef EXC_NRF24 
  
  #include <RF24Network.h>
  #include <RF24.h>
#endif 
#define EXC_InputOn  LOW

/******************************USER CODE LIBRARY START******************************************************************/

/*************************************************************/
//USER SETTING


/******************************USER CODE LIBRARY END********************************************************************/
#ifdef THERMOSTAT_DTH22 
  #define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
  #define DHTPIN 2 //datos sensor dht pin 2
  DHT dht(DHTPIN, DHTTYPE);
  float HumedadDHT=0; //Variable para almacenar sonda de humedad.
  float TemperatureDHT=0; // Variable para almacenar sonda Temperatura
  boolean AveriaDHT=false;
  byte DhtErrorCount=0;
#endif 
#ifdef THERMOSTAT_DS18B20_NUMBER 

  #define TEMPERATURE_PRECISION 9
#define ONE_WIRE_BUS 16
  DeviceAddress Ds18B20Addres[THERMOSTAT_DS18B20_NUMBER] ={{0x28,0xFF,0x33,0x75,0x78,0x04,0x00,0x5A},{0x28,0xFF,0xDC,0x8F,0x61,0x15,0x02,0xD5},{0x28,0xFF,0xE0,0x2F,0x77,0x04,0x00,0xD6}};
  OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  DallasTemperature sensorTemp(&oneWire);// Pass our oneWire reference to Dallas Temperature.   

float Temperature[]={20,21,22};
 
#endif 
  const byte Histeresis =5;
  
  
#ifdef EXC_LED_IR  
  IRsend irsend;
#endif 

#ifdef EXC_IR_RECIVE
  IRrecv irrecv(19);//El 19 corresponde con el pin de arduino, cambiar para utilizar otro
  decode_results results;
#endif 

#if defined (EXC_TRANSMITER_433)  || defined (EXC_RECEIVER_433)
  RCSwitch mySwitch = RCSwitch();
#endif 
#ifdef EXC_TRANSMITER_433
  #define EXC_433enableTransmit 
  #define EXC_433setPulseLength 320
  #define EXC_setProtocol 1
  #define EXC_433setRepeatTransmit 15
#endif 
#ifdef ENABLE_SMODBUS
  const byte TxPin=;
  boolean ModbusOuts[32];
  //////////////// registers of your slave ///////////////////
  
  enum 
  {     
    // just add or remove registers and your good to go...
    // The first register starts at address 0
    OUT_REG1,OUT_REG2,      
    IN1_REG1,IN1_REG2,IN1_REG3,IN1_REG4,IN1_REG5,IN1_REG6,SENSOR1_REG1,SENSOR1_REG2,   
    
    //IN2_REG1,IN2_REG2,IN2_REG3,IN2_REG4,IN2_REG5,IN2_REG6,SENSOR2_REG1,SENSOR2_REG2, //slave 2
    //IN3_REG1,IN3_REG2,IN3_REG3,IN3_REG4,IN3_REG5,IN3_REG6,SENSOR3_REG1,SENSOR3_REG2, //slave 3
    //IN4_REG1,IN4_REG2,IN4_REG3,IN4_REG4,IN4_REG5,IN4_REG6,SENSOR4_REG1,SENSOR4_REG2, //slave 4
    
    HOLDING_REGS_SIZE 
  };
  
  unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array
  unsigned int OldholdingRegs[HOLDING_REGS_SIZE];
  ////////////////////////////////////////////////////////////

#endif
#ifdef EXC_LCD
   LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 
#endif
#ifdef EXC_NRF24

  const uint8_t RF24channel = 90;
  const uint16_t dest_address =00; 
  const uint16_t this_node =02; 
  //uint8_t NODE_ADDRESS = 0;  // Use numbers 0 through to select an address from the array
  //RF24 (cepin, cspin) 
  RF24 radio(CE_nRF24L01,SS_nRF24L01);                              // CE & CS pins to use (Using 7,8 on Uno,Nano)
  RF24Network network(radio); 


 boolean NrfOuts[32];
 
const byte NrfRegSize = 8;
const byte DevNumber = 1;
 
 unsigned short NrfInRegs[DevNumber][8]; 
 unsigned int OldNrfInRegs[DevNumber][8]; 
 
 const int NrfRS = sizeof(NrfInRegs[0]);


const byte UserRegNumber=2;
  unsigned short NRFRegs[UserRegNumber];
  unsigned short OldNRFRegs[UserRegNumber];
  const int NrfRegS = sizeof(NRFRegs);
  unsigned int NrfCiclos=0;
  boolean Escucha=false;
  unsigned long TimSend =0;
  byte TimRcv=0;
        
#endif 
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
//ZONA DE CONFIGURACIONES 
//SETTINGS ZONE
//Define numero de entradas salidas 
//Configuracion Red
//Activa o desactiva el perro guardian
/******************************************************************************************************************************/

//SETTINGS ZONE
//Defines number of inputs and outputs
//Network configuration
//Set or Restet Daylight saving time o DST
//Set or Reset watchdog
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
//Activa o desactiva cambio hora automatico invierno verano
//Set or Restet Daylight saving time o DST
//El modo dts esta configurado para europa
//Dts mode is set to europe
//Para otros paises configure funcion CargaHora()
//For other countries configure  function  CargaHora()
const boolean Enable_DaylightSavingTime  = true; 



//Define pines de Entradas y salidas
//Inputs pin and outs pin

  

const byte PinInput[]={23,25,27,29,31,33,35,37,39,41,43};
byte PinOutput[]={22,24,26,28,30,32,34,36,38,40,42,44,46};


/***********************************************************************************************************************/
/***********************************************************************************************************************/
//VARIABLES PARA CONTROL ESTADO CIRUCITOS
//EL RETORNO DE ESTADO SE RECOGE POR ENTRADAS DIGITALES.
/***********************************************************************************************************************/
/***********************************************************************************************************************/  


//CONFIGURACION DE RED
//Direccion IP ES MUY PROBABLE QUE TENGAS QUE AJUSTARLO A TU RED LOCAL
//Cambia la ip por una dentro de tu rango, tambien debes actualizarla en tu aplicacion android
//Tambien puedes cambiar el puerto, por defecto 5000

// NETWORK CONFIGURATION
// IP Address, ADJUST TO YOUR LOCAL NETWORK
//If you change the IP address will have to adjust in android application
//If you change the Local Port address will have to adjust in android application
//WIRELESS CONFIGURATION
#ifdef EXC_WIFI_SHIELD 
  bool ApOn=true; 
  enum Net_Security {OPEN,WEP,WPA,A_POINT};
  Net_Security Net_Type = WPA;
  char ssid[] = ""; //  your network SSID (name) 
  char pass[] = ""; //  // your network password (use for WPA, or use as key for WEP) 
  int keyIndex = 0;// your network key Index number (needed only for WEP)
  
  byte TimConexion=0;
  int status = WL_IDLE_STATUS;     // the Wifi radio's status
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
  WiFiUDP Udp;
  #ifdef ExControlMail 
    WiFiClient client;  
  #endif  
#else
  // buffers para recepcion y envio de datos
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; 
   // Instanacia Ethernet UDP Para enviar y recibir paqueteP
  EthernetUDP Udp;
  #ifdef ExControlMail 
    EthernetClient client;
  #endif
#endif


byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0x26, 0x20};
IPAddress ip(192,168,1,115);
unsigned int localPort = 5002;

const boolean SecureConnection=true;


#ifdef EXC_I2C_BOARD
   struct I2cBoard {byte Inputs; byte Outputs;};
   I2cBoard IoBoards[EXC_I2C_BOARD];
   byte ov[EXC_I2C_BOARD];
#endif
#ifdef EXC_SERVER
  IPAddress ServerIP(192,168,1,200);
  const int SeverPort= 192;
  struct RemotePacket {boolean Send; char Data[16]; unsigned long LastSend;};
  const byte RemotePacketNumber= 0;
  RemotePacket RemotePackets[RemotePacketNumber];
  byte InternalComPacket=0;
  boolean Deslastre =false;
  byte TimUdp=190;
#endif
//CONFIGURACION DE HISOTORICOS
#ifdef SD_CARD
  boolean SdOk=true;
  File SdFile;
  const byte  MinutesHistorico=15;
  byte MinutesLstHist=0;
#endif

// Circuitos , entradas y salidas
//Circutis, inputs and outputs.

struct Sensor {byte Type;byte Device_Number;short Value;boolean Damaged;};
struct Entrada {byte Type;byte InState;unsigned long LastTimeInput;};
struct Circuit {byte Type;boolean Out1_Value;boolean Out2_Value;byte Device_Number;byte Value;byte CopyRef; byte OldValue;};




byte TypeSensors[] ={Sensor_Temperature,Sensor_Temperature,Sensor_Temperature,Sensor_Generic};
byte TypeCircuits[] ={Ado_Digital,Ado_Digital,Ado_Digital,Ado_Digital,Ado_Digital,Ado_Digital,Ado_Digital,Ado_Digital,Ado_Digital,Ado_Digital,Ado_Digital,HomeTemperature,HomeTemperature,HomeTemperature,Puerta,Puerta,SensorView};
byte TypeInputs[] ={Button,Button,Button,Button,Button,Button,Button,Button,Button,Button,Button};
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
//FIN DE ZONAS DE CONFIGURACIONES
//END SETTINGS ZONE
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
#ifdef EXC_ARDUINO_MEGA
  #define SS_ETHERNET 53 //53 for mega, for other set pin to 10
  #define SS_SD 4
  #define SS_UNO 10
  
#endif



const byte Number_Input=sizeof(TypeInputs);
const byte Number_Output=sizeof(PinOutput);
const byte Number_Circuit=sizeof(TypeCircuits);
const byte Number_Sensor=sizeof(TypeSensors);

Sensor  Sensors[Number_Sensor];
Circuit circuits[Number_Circuit];
Entrada Inputs[Number_Input];

#ifdef ExControlMail
  byte EspRfrIp = 0;
#endif 


//Varibles Reloj
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year,minutoMemory,TipoDia;
boolean HoraRetrasa;
unsigned long TimNow=0;
unsigned long ExTimer=0;
byte CountMsg=0;
byte CountSg=0;
short CondiVer=0;
#ifdef EXC_RGB_LED
  byte RGBredVal = 255;
  byte RGBblueVal = 0;
  byte RGBgreenVal =0;
  byte RGBSpeed=4;
  byte RGBrandomCount=0;
#endif
//Variables Control Alarmas
byte  IntCom=0;
byte Alarms[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

#ifdef EXC_NumeroPersianas
  
  byte PosicionPersiana[EXC_NumeroPersianas];  //Controla la posicion de la persiana % Subida
  byte LocalizadorPersiana[EXC_NumeroPersianas];  //Controla posicion persiana en array de circuitos
  
  
  unsigned long TiempoMovPersiana[EXC_NumeroPersianas];  //Tiempo de mov desde el ultimo refresco
  
  //Memoria de tiempos y posicion respecto a tiempo
  unsigned long TiempoPosPersianaUp[EXC_NumeroPersianas];  //Posicion persiana en subida
  unsigned long TiempoPosPersianaDown[EXC_NumeroPersianas];  //Posicion persiana en Bajada
  unsigned long TimUpPersiana[EXC_NumeroPersianas];  //Tiempo en MicrosSg subida persiana
  unsigned long TimDowPersiana[EXC_NumeroPersianas];  //Tiempo en MicrosSg bajada persiana
  
  //Variables para salidas y entradas
  boolean OutUpPersiana[EXC_NumeroPersianas];  //Boleana para activar salida persiana
  boolean OutDowPersiana[EXC_NumeroPersianas];  //Boleana para activar salida persiana
  boolean InUpPersiana[EXC_NumeroPersianas];  //Boleana pulsador subida Persiana
  boolean InDowPersiana[EXC_NumeroPersianas];  //Boleana pulsador bajada Persiana
#endif
boolean Condicionados[10];              //Guarda el estado de los condicionados
word Consignas[10];                     //Guarda el valor de las consignas
boolean Connecting=false;
boolean ForcingSpecialDay1=false;
boolean ForcingSpecialDay2=false;
unsigned long TimAcDet=500;
byte SegUpdtHora=0;

/******************************GLOBAL VARIABLES ZONE******************************************************************/

/*************************************************************/
//USER SETTING
byte LcdTimeCount=0 ;
#define PIN_ECHO   2

long distancia, duracion;
int estado_anterior, sincro = 3, estado_sincro = HIGH, pres = 6, elev = 5, litros;
boolean estado_pres = HIGH,estado_elev = HIGH;

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
//ZONA DE CONFIGURACIONES 
//SETTINGS ZONE

/******************************************************************************************************************************/
void UserSetup() { //EQUIVALENT ARDUINO SETUP  FUNCTION

//AUTO GENERATED CODE
/*************************************************************/
  Sensors[0].Device_Number =0;//Interior
  Sensors[1].Device_Number =1;//Exterior
  Sensors[2].Device_Number =2;//Tablero
  Sensors[3].Device_Number =240;//Sensor de Agua
  circuits [11].Device_Number =0;//Interior
  circuits [12].Device_Number =1;//Exterior
  circuits [13].Device_Number =2;//Tablero
  circuits [16].Device_Number =3;//Tanque de Agua

/*************************************************************/
//USER SETTING
 pinMode(PIN_ECHO,INPUT);
  pinMode(sincro,INPUT_PULLUP);
  pinMode(pres,INPUT_PULLUP);
  
//EQUIVALENT ARDUINO SETUP  FUNCTION
//EQUIVALENTE A FUNCION SETUP DEL PROGRAMA ARDUINO

}

void UserLoop(){ //EQUIVALENT ARDUINO LOOP FUNCTION

/*************************************************************/
//USER SETTING
long distancia;
  long duracion;
  estado_sincro = digitalRead(sincro);
  if(estado_sincro == LOW)
    {
      
      duracion = pulseIn (PIN_ECHO , HIGH);
      distancia = duracion /58.2;
     litros = 1184- (distancia *8.4654);
     Serial.print ("Distancia: ");
     Serial.print (distancia );
     Serial.println ("cm");
     Serial.print ("Litros: ");
     Serial.println (litros );
 
    
      Serial.println ("");
      delay(100);
      estado_pres = HIGH;
    }

/*************************************************************/
//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//END AUTO GENERATED CODE
/*************************************************************/

  
//EQUIVALENT ARDUINO LOOP  FUNCTION
//EQUIVALENTE A FUNCION LOOP DEL PROGRAMA ARDUINO
}
void LoopNew100MillisSg(){//This event occurs every 100Millis Sg.

/*************************************************************/
//USER SETTING

  
  //Este evento se produce cada 100 milisegundos
  //This event occurs every 100Millis Sg.
}
void LoopNewSecond(){//This event occurs every second

//AUTO GENERATED CODE
/*************************************************************/
  Sensors[0].Value =Temperature[0]*10;//Interior
  Sensors[1].Value =Temperature[1]*10;//Exterior
  Sensors[2].Value =Temperature[2]*10;//Tablero

/*************************************************************/
//USER SETTING
Sensors[3].Value = litros;
Sensors[5].Value = estado_pres;
Sensors[6].Value = estado_elev;

if (circuits[24].Value>=1){circuits[24].Value=0;} //linea 1 
SetRelay(PinOutput[21],circuits[24].Out1_Value); //linea 2
if (circuits[24].Value>=1){circuits[24].Value=circuits[24].Value + 1;} //linea 3

if (circuits[25].Value>=1){circuits[25].Value=0;} //linea 1 
SetRelay(PinOutput[22],circuits[25].Out1_Value); //linea 2
if (circuits[25].Value>=1){circuits[25].Value=circuits[25].Value + 1;} //linea 3

if (LcdTimeCount==0){
    lcd.clear();
    writeLCD(0, "%D %2/%0/%2 %H:%M",dayOfMonth, month, year);
    writeLCD(1,"#2 Interior     %f#4",(float)Sensors[0].Value/10);
    writeLCD(2,"#2 Exterior     %f#4",(float)Sensors[1].Value/10);
    writeLCD(3,"#2 Cuartito     %f#4",(float)Sensors[2].Value/10);
       Serial.println("Pantalla 1");
    }
    if (LcdTimeCount==10){
     lcd.clear();
    writeLCD(0, "#6 Lampara Living   %b",circuits[0].Value );
    writeLCD(1, "#6 Lampara Comedor  %b",circuits[1].Value );
    writeLCD(2, "#6 Lampara Desayuna %b",circuits[2].Value );
    writeLCD(3, "#6 Lampara Cocina   %b",circuits[3].Value );
    Serial.println("Pantalla 2");
    }
    if (LcdTimeCount==20){
    lcd.clear();
    writeLCD(0, "#6 Lampara Mesada   %b",circuits[4].Value );
    writeLCD(1, "#6 Lampara Recibido %b",circuits[5].Value );
    writeLCD(2, "#6 Lampara Cuartito %b",circuits[6].Value );
    writeLCD(3, "#6 Lampara Esc.P.Ba %b",circuits[7].Value ); 
    Serial.println("Pantalla 3");
    }

    if (LcdTimeCount==30){
    lcd.clear();
    writeLCD(0, "#6 Lampara Calle    %b",circuits[8].Value );
    writeLCD(1, "#6 Lampara Sal.Bano %b",circuits[9].Value );
    writeLCD(2, "#6 Lampara Esc.P.Al %b",circuits[10].Value );
    writeLCD(3, "#7    Leandrouno    #7" ); 
    Serial.println("Pantalla 4");
    }
	if (LcdTimeCount==40){
    lcd.clear();
    writeLCD(0, "Litros de Agua:%4", Sensors[3].Value , litros);
    writeLCD(1, "Presurizadora :", Sensors[5].Value ,estado_pres);
    writeLCD(2, "Elevadora     :", Sensors[6].Value ,estado_elev);
    writeLCD(3, "#7    Leandrouno    #7" ); 
    Serial.println("Pantalla 5");
    }
	LcdTimeCount++;//Incrementamos su valor
    if (LcdTimeCount==4){LcdTimeCount=10;}//Cuando llegamos a 4 cambiamos de pantalla
	if (LcdTimeCount==14){LcdTimeCount=20;}//Cuando llegamos a 4 cambiamos de pantalla
	if (LcdTimeCount==24){LcdTimeCount=30;}//Cuando llegamos a 4 cambiamos de pantalla
    if (LcdTimeCount==34){LcdTimeCount=40;}//Cuando llegamos a 4 cambiamos de pantalla
	if (LcdTimeCount==44){LcdTimeCount=0;}//Cuando llegamos a 4 cambiamos de pantalla


}
void Loop30Sg(){//This event occurs every 30 second

/*************************************************************/
//USER SETTING
elevadora();
presurizadora();
  
  //Este evento se produce cada 30sg.
  //This event occurs every 30SG.
}
void NewMinute(){//This event occurs every minute.

/*************************************************************/
//USER SETTING

   
  //Este evento se produce cada cada minuto.
  //This event occurs every minute.
}
/******************************************************************************************************************************/
 /***********************************************************************************************************************/
/***********************************************************************************************************************/
//EVENTOS CONTROL ENTRADAS SALIDAS
//INPUT OUTPUT CONTROL EVENTS

//CUATRO EVENTOS PARA ENTRADAS DIGITALES
//CONMUTADOR CAMBIA VALOR
//PULSACION CORTA
//PULSACION LARGA, MAYOR DE 0.5 SEGUNDOS
//FINAL PULSACION LARGA

// FOUR EVENTS FOR DIGITAL INPUTS
//VALUE CHANGE SWITCH
// PRESS SHORT
// PRESS LONG, OVER 0.5 SECONDS
// LONG PRESS END.

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void SwicthStateChange(int NumberInput, int value){

/*************************************************************/
//USER SETTING

//AUTO GENERATED CODE
/*************************************************************/
/*************************************************************/
//END GENERATED CODE
/*************************************************************/
  /*****************************************************************/
  //Este evento se produce cuando un conmutador cambia posicion
  // This event occurs with swicth change state.
  //dos parametros
  //two parameters
  //Number input--numero de entrada
  //Value, if input is HIGH value = HIGH, If value=low then Value=Low
  //Value, si la entrada esta en nivel alto el valor es HIGH si es bajo el valor es LOW
  //Tambien puede acceder desde cualquier punto del codigo al valor de la entrada con SwicthState[]
  //You can know input state anywhere using SwictState[]
  

  #ifdef EXC_DEBUG_MODE   
   Serial.print("Swict State Change, input ");
   Serial.print(NumberInput);
    if (value==LOW){Serial.println(" Off");}else{Serial.println(" On");}
  #endif
}
void ShortInput(int NumberInput){
/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.
//AUTO GENERATED CODE
/*************************************************************/

  //CIRCUITO NUMERO 0
  //Lampara Living
  if (NumberInput==0){if (circuits[0].Value > 0){circuits[0].Value=0;}else{circuits[0].Value=1;}}
  //CIRCUITO NUMERO 1
  //Lampara Comedor
  if (NumberInput==1){if (circuits[1].Value > 0){circuits[1].Value=0;}else{circuits[1].Value=1;}}
  //CIRCUITO NUMERO 2
  //Lampara Desayunador
  if (NumberInput==2){if (circuits[2].Value > 0){circuits[2].Value=0;}else{circuits[2].Value=1;}}
  //CIRCUITO NUMERO 3
  //Lampara Cocina
  if (NumberInput==3){if (circuits[3].Value > 0){circuits[3].Value=0;}else{circuits[3].Value=1;}}
  //CIRCUITO NUMERO 4
  //Lampara Mesada
  if (NumberInput==4){if (circuits[4].Value > 0){circuits[4].Value=0;}else{circuits[4].Value=1;}}
  //CIRCUITO NUMERO 5
  //Lampara Recibidor
  if (NumberInput==5){if (circuits[5].Value > 0){circuits[5].Value=0;}else{circuits[5].Value=1;}}
  //CIRCUITO NUMERO 6
  //Lampara Cuartito
  if (NumberInput==6){if (circuits[6].Value > 0){circuits[6].Value=0;}else{circuits[6].Value=1;}}
  //CIRCUITO NUMERO 7
  //Lampara Escalera Planta Baja
  if (NumberInput==7){if (circuits[7].Value > 0){circuits[7].Value=0;}else{circuits[7].Value=1;}}
  //CIRCUITO NUMERO 8
  //Lampara Calle
  if (NumberInput==8){if (circuits[8].Value > 0){circuits[8].Value=0;}else{circuits[8].Value=1;}}
  //CIRCUITO NUMERO 9
  //Lampara Salida de Baño
  if (NumberInput==9){if (circuits[9].Value > 0){circuits[9].Value=0;}else{circuits[9].Value=1;}}
  //CIRCUITO NUMERO 10
  //Lampara  Escalera Planta ALta
  if (NumberInput==10){if (circuits[10].Value > 0){circuits[10].Value=0;}else{circuits[10].Value=1;}}
/*************************************************************/
//END GENERATED CODE
/*************************************************************/

/*************************************************************/
//USER SETTING

/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.
//*************************************************************/

  #ifdef EXC_DEBUG_MODE   
   Serial.print("Short Input End ");
   Serial.println(NumberInput);
   if (NumberInput<Number_Input){Serial.print(" pin ");	Serial.println(PinInput[NumberInput]);	}  
  #endif
  

}
void LongInputEnd(int NumberInput){
/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.
//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//END GENERATED CODE
/*************************************************************/

/*************************************************************/
//USER SETTING

//Este evento se produce cuando la pulsación larga acaba..
//This event occurs with  long press end.

  #ifdef EXC_DEBUG_MODE   
    Serial.print("Long Input End ");
    Serial.println(NumberInput);
    if (NumberInput<Number_Input){Serial.print(" pin ");Serial.println(PinInput[NumberInput]);} 
  #endif

  /*****************************************************************/
  //FINAL DE PULSACION LARGA
  //LONG PRESS END, EVENT
  // This event occurs with end a long press.
  /*****************************************************************/
  
}
void LongInput(int NumberInput){
/*************************************************************/
//Este evento se produce con una pulsación corta..
//This event occurs with a short press.
//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//END GENERATED CODE
/*************************************************************/

/*************************************************************/
//USER SETTING

/*************************************************************/
//Este evento se produce con una pulsación larga comienca..
//This event occurs with a long press start.
/*************************************************************/
  #ifdef EXC_DEBUG_MODE   
    Serial.print("Long Input  ");
    Serial.println(NumberInput);
    if (NumberInput<Number_Input){Serial.print(" pin ");Serial.println(PinInput[NumberInput]);} 
  #endif 
}

//Set or reset relay output
void SetRelay(byte Pin, boolean On){if (On){digitalWrite(Pin,HIGH);}else{digitalWrite(Pin,LOW);}}
  
void OutControl(){

/*************************************************************/
//USER SETTING

/*************************************************************/
//Activamos los reles de control. Activate control relays.
//AUTO GENERATED CODE
/*************************************************************/

  int e;
  //CIRCUITO NUMERO 0
  //Lampara Living
  //Out 1
  SetRelay(PinOutput[0],circuits[0].Out1_Value);

  //CIRCUITO NUMERO 1
  //Lampara Comedor
  //Out 1
  SetRelay(PinOutput[1],circuits[1].Out1_Value);

  //CIRCUITO NUMERO 2
  //Lampara Desayunador
  //Out 1
  SetRelay(PinOutput[2],circuits[2].Out1_Value);

  //CIRCUITO NUMERO 3
  //Lampara Cocina
  //Out 1
  SetRelay(PinOutput[3],circuits[3].Out1_Value);

  //CIRCUITO NUMERO 4
  //Lampara Mesada
  //Out 1
  SetRelay(PinOutput[4],circuits[4].Out1_Value);

  //CIRCUITO NUMERO 5
  //Lampara Recibidor
  //Out 1
  SetRelay(PinOutput[5],circuits[5].Out1_Value);

  //CIRCUITO NUMERO 6
  //Lampara Cuartito
  //Out 1
  SetRelay(PinOutput[6],circuits[6].Out1_Value);

  //CIRCUITO NUMERO 7
  //Lampara Escalera Planta Baja
  //Out 1
  SetRelay(PinOutput[7],circuits[7].Out1_Value);

  //CIRCUITO NUMERO 8
  //Lampara Calle
  //Out 1
  SetRelay(PinOutput[8],circuits[8].Out1_Value);

  //CIRCUITO NUMERO 9
  //Lampara Salida de Baño
  //Out 1
  SetRelay(PinOutput[9],circuits[9].Out1_Value);

  //CIRCUITO NUMERO 10
  //Lampara  Escalera Planta ALta
  //Out 1
  SetRelay(PinOutput[10],circuits[10].Out1_Value);

  //CIRCUITO NUMERO 14
  //Puerta Negra
  //Out 1
  SetRelay(PinOutput[11],circuits[14].Out1_Value);

  //CIRCUITO NUMERO 15
  //Puerta Blanca
  //Out 1
  SetRelay(PinOutput[12],circuits[15].Out1_Value);

/*************************************************************/
//END GENERATED CODE
/*************************************************************/

}

String RunCommand(byte CommandNumber){

/*************************************************************/
//USER SETTING

  //Este evento se produce cuando se ejecuta un comando desde el app
  //This event occurs when a command is executed from the app
   #ifdef EXC_DEBUG_MODE   
    Serial.print("Command Nª");Serial.println(CommandNumber);     
  #endif
  
  //Enabled this line for send infrared 
   #ifdef EXC_LED_IR
     SendIr(CommandNumber);
   #endif
  
 return "COMPLETED";
}
void CommonOrders(byte CommandNumber){

/*************************************************************/
//USER SETTING

  //Este evento se produce cuando se ejecuta un comando desde el app
  //This event occurs when a command is executed from the app
  
  #ifdef EXC_DEBUG_MODE   
    Serial.println(CommandNumber);Serial.println(CommandNumber);             
  #endif
}
  //Free text tool
  //Herramienta de texto libre
  //parameter number is the number of scren
  //El parametro number indica el numero de lina en la app
String FreeText(byte Number){

/*************************************************************/
//USER SETTING



  return "RESERVA"; //No borrar, dont delete this line
}
void PersonalFunctions(byte Number){

/*************************************************************/
//USER SETTING

  
}
String GetAlarmsName(byte Number){
/*************************************************************/
//AUTO GENERATED CODE
/*************************************************************/

/*************************************************************/
//END AUTO GENERATED CODE
/*************************************************************/

  
  
    return "RESERVA";
}
void  Scene_Selected(byte Number){

/*************************************************************/
//USER SETTING

  
}
void InternalPacketIn(byte Data1,byte Data2,byte Data3,byte Data4,byte Data5,byte Data6,byte Data7,byte Data8,byte Data9,byte Data10){

/*************************************************************/
//USER SETTING

   #ifdef EXC_DEBUG_MODE   
       Serial.print("User packet in= ");Serial.print(Data1);Serial.print(Data2);Serial.print(Data3);Serial.print(Data4);Serial.print(Data5);Serial.print(Data6);Serial.print(Data7);Serial.print(Data8);Serial.print(Data9);Serial.println(Data10);          
   #endif
}

void Mhz433In(int value){

/*************************************************************/
//USER SETTING


}
#ifdef EXC_LCD
  void PrintMyLcd(){
//AUTO GENERATED CODE
/*************************************************************/


}
#endif
#ifdef EXC_NRF24
void NRF24_CommOk(boolean Send){//Nrf24l01 complete communication

/*************************************************************/
//USER SETTING


}
#endif
void AutomaticDST(){
//DST time change (default for Europe)
  if(month==3 && dayOfMonth >= 25 && dayOfWeek == 7 && hour==2){
    hour = 3;
    setDateCLOCK(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
  }
  if(month==10 && dayOfMonth >= 25 && dayOfWeek == 7 && hour==3){
    if (HoraRetrasa==false){
      HoraRetrasa=true;
      hour = 2; 
      setDateCLOCK(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
    }
  }
/*************************************************************/

}

/*************************************************************/
//USER FUNCTIONS
void presurizadora()
{
  estado_pres = digitalRead (pres);
  if (estado_pres == LOW){

     Serial.println ("Presurizadora OFF"); 
    }  
    else
      {
      
       Serial.println ("Presurizadora ON");
      }     
  }

  void elevadora()
{
  estado_elev = digitalRead (elev);
  if (estado_elev == LOW){

     Serial.println ("Elevadora OFF"); 
    }  
    else
      {
      
       Serial.println ("Elevadora ON");
      }     
  }
