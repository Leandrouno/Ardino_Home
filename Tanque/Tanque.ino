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
#define ExControlPass "*******"

//****************************************************
//CONFIGURACION EQUIPOS INTALADOS, TERMOSTATOS, ENCHUFES RADIOFRECUENCIA 433MHZ, INFARROJOS.
//EQUIPMENT CONFIGURATION , THERMOSTAT, RADIO 433MHZ, INFARROJOS.



#define EXC_LCD
//#define EXC_LED_IR
//#define EXC_IR_RECIVE
//#define EXC_RECEIVER_433
//#define EXC_TRANSMITER_433
//#define EXC_RGB_LED
//#define THERMOSTAT_DS18B20_NUMBER
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
#define ONE_WIRE_BUS 
  DeviceAddress Ds18B20Addres[THERMOSTAT_DS18B20_NUMBER] ={{0x28,0xAB,0x2A,0x46,0x04,0x00,0x00,0x88}};
  OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  DallasTemperature sensorTemp(&oneWire);// Pass our oneWire reference to Dallas Temperature.   

  float Temperature[]={20};
 
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
   LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 
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

  

const byte PinInput[]={23,25,27};
byte PinOutput[]={14,15};


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


byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0x26, 0x22};
IPAddress ip(192,168,1,201);
unsigned int localPort = 5000;

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




byte TypeSensors[] ={Sensor_Generic};
byte TypeCircuits[] ={Valvula,Valvula,SensorView,Ado_Digital};
byte TypeInputs[] ={Button,Button,Button};
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
#define PIN_ECHO   2
#define PIN_TRIGER 3

int sincro = 5;

int R1 = 7; //Estado Elevadora
int R2 = 8; // Estado Presurizadora

int Led1 =48;  //Nivel 1   Rojo
int Led2 =46;  //Nivel 2   Rojo
int Led3 =44;  //Nivel 3   Amarillo
int Led4 =42;  //Nivel 4   Amarillo
int Led5 =40;  //Nivel 5   Amarillo
int Led6 =38;  //Nivel6    Verde
int Led7 =36; //Nivel 7   Verde
int Led8 =34; //Nivel 8   Verde

int litros;

int estado1 = 0;
int pulsa_led = 5;
int estado_led= 0;

long distancia, duracion, seg_actual, seg_anterior = 0;
int estado = HIGH;

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
  Sensors[0].Device_Number =240;//Sensor Agua
  circuits [2].Device_Number =0;//Sensor Agua

/*************************************************************/
//USER SETTING
pinMode(PIN_TRIGER,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  pinMode(Led1,OUTPUT);
  pinMode(Led2,OUTPUT);
  pinMode(Led3,OUTPUT);
  pinMode(Led4,OUTPUT);
  pinMode(Led5,OUTPUT);
  pinMode(Led6,OUTPUT);
  pinMode(Led7,OUTPUT);
  pinMode(Led8,OUTPUT);
  pinMode(R1,OUTPUT);
  pinMode(R2,OUTPUT);
  pinMode(sincro,OUTPUT);
  pinMode(pulsa_led,INPUT);  
  digitalWrite(sincro,HIGH);
 
   digitalWrite(sincro,LOW);
   digitalWrite ( PIN_TRIGER, LOW);
   delayMicroseconds (2);
   digitalWrite ( PIN_TRIGER, HIGH);
   delayMicroseconds (10);
   duracion = pulseIn (PIN_ECHO , HIGH); 
   distancia = duracion /58.2;
   litros = 1184- (distancia *8.4654);
   Serial.print ("Distancia: ");
   Serial.print (distancia );
   Serial.println ("cm");
   Serial.print ("Litros: ");
   Serial.println (litros );
   Serial.println ("");
   digitalWrite(sincro,HIGH);
  
//EQUIVALENT ARDUINO SETUP  FUNCTION
//EQUIVALENTE A FUNCION SETUP DEL PROGRAMA ARDUINO

}

void UserLoop(){ //EQUIVALENT ARDUINO LOOP FUNCTION

/*************************************************************/
//USER SETTING
if (circuits[3].Value > 0){
	  estado_led= 1;
	 
	  }
	  else if (circuits[3].Value == 0)
      {
		  estado_led= 0;
      
      }    
  
  if (estado_led== 1){
	  led();
	//  Serial.println ("Led ON");
  }
  else{
	  led_of();
	  // Serial.println ("Led OFF");
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

/*************************************************************/
//USER SETTING
Sensors[0].Value = litros;


}
void Loop30Sg(){//This event occurs every 30 second

/*************************************************************/
//USER SETTING
digitalWrite(sincro,LOW);
   digitalWrite ( PIN_TRIGER, LOW);
   delayMicroseconds (2);
   digitalWrite ( PIN_TRIGER, HIGH);
   delayMicroseconds (10);
   duracion = pulseIn (PIN_ECHO , HIGH); 
   distancia = duracion /58.2;
   litros = 1184- (distancia *8.4654);
   Serial.print ("Distancia: ");
   Serial.print (distancia );
   Serial.println ("cm");
   Serial.print ("Litros: ");
   Serial.println (litros );
   Serial.println ("");
   digitalWrite(sincro,HIGH);


Sensors[0].Value = litros;

if (Condicionados[0]==true){                                           // Si la condicion 0 Esta Activada
	if (circuits[0].Value == 0){                                      // Si la Presurizadora Esta Apagada
	if  (Sensors[0].Value <= Consignas[1]) {                         // Si Litros es Menor o igual que Elev_Min
	circuits[0].Value=249;                                          // La Elevadora Pasa a valor 249 ( Automatico)
	Serial.println ("Elevadora ON");
		}			 
	}


else if ((Sensors[0].Value >= Consignas[0])&&(circuits[0].Value >0)){        // sino, Si((litros es mayor o igual Elev_Max)&&(Elevadora Encendida) )
      circuits[0].Value=0; 													// La Elevadora se Apaga
	Serial.println ("Elevadora OFF");
	}
}


if (Condicionados[1]==true){                                           // Si la condicion 1 Esta Activada
	 if (circuits[1].Value == 0){                                     // Si la Elevadora Esta Apagada
		 if  (Sensors[0].Value >= Consignas[3]) {                    // Si Litros es Mayor o igual que Presu_minimo
		 circuits[1].Value=249;                                     // La Presurizadora Pasa a valor 249 ( Automatico)
		 Serial.println ("Presurizadora ON");
		 
		}
	

	
       else {	                                                         // Sino
	         circuits[1].Value=0;                                       // La Elevadora se apaga
             Serial.println ("Presurizadora OFF");
             }	
			 }
}


Serial.println ("");
Serial.print(" Elevadora Maximo {0} = ") ;
Serial.println(Consignas[0]);

Serial.print(" Elevadora Minimo {1} = ") ;
Serial.println(Consignas[1]);

Serial.print(" Presurizadora Maximo {2} = ") ;
Serial.println(Consignas[2]);

Serial.print(" Presurizadora Minimo {3} = ") ;
Serial.println(Consignas[3]);
     
Serial.print("Estado Led: ");
Serial.println(estado_led);
Serial.println ("");
  
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
  //Elevadora
  if (NumberInput==1){if (circuits[0].Value > 0){circuits[0].Value=0;}else{circuits[0].Value=1;}}
  //CIRCUITO NUMERO 1
  //Presurizadora
  if (NumberInput==2){if (circuits[1].Value > 0){circuits[1].Value=0;}else{circuits[1].Value=1;}}
  //CIRCUITO NUMERO 3
  //Indicador Led
  if (NumberInput==0){if (circuits[3].Value > 0){circuits[3].Value=0;}else{circuits[3].Value=1;}}
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
  //Elevadora
  //Out 1
  SetRelay(PinOutput[0],circuits[0].Out1_Value);

  //CIRCUITO NUMERO 1
  //Presurizadora
  //Out 1
  SetRelay(PinOutput[1],circuits[1].Out1_Value);

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
void led() {
int nivel[8] = {0,0,0,0,0,0,0,0}, maxi = 0;
if (distancia < 30) {digitalWrite(Led8,HIGH); nivel[7] = 8;} else digitalWrite(Led8,LOW);
if (distancia < 40) {digitalWrite(Led7,HIGH); nivel[6] = 7;} else digitalWrite(Led7,LOW);
if (distancia < 50) {digitalWrite(Led6,HIGH); nivel[5] = 6;} else digitalWrite(Led6,LOW);
if (distancia < 60) {digitalWrite(Led5,HIGH); nivel[4] = 5;} else digitalWrite(Led5,LOW);
if (distancia < 70) {digitalWrite(Led4,HIGH); nivel[3] = 4;} else digitalWrite(Led4,LOW);
if (distancia < 80) {digitalWrite(Led3,HIGH); nivel[2] = 3;} else digitalWrite(Led3,LOW);
if (distancia < 85) {digitalWrite(Led2,HIGH); nivel[1] = 2;} else digitalWrite(Led2,LOW);
if (distancia < 90) {digitalWrite(Led1,HIGH); nivel[0] = 1;} else digitalWrite(Led1,LOW);
for( int i = 0; i < 8 ; i++)
  if(nivel[i] > maxi)
     maxi = nivel[i];

}

void led_of() {

digitalWrite(Led8,LOW);
digitalWrite(Led7,LOW);
digitalWrite(Led6,LOW);
digitalWrite(Led5,LOW);
digitalWrite(Led4,LOW);
digitalWrite(Led3,LOW);
digitalWrite(Led2,LOW);
digitalWrite(Led1,LOW);

}
