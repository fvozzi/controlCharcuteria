//#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Bounce2.h>
#include <dht.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); 
//DHT dht(2, DHT22);

dht DHT;

//Debug mode 1 ON
const int debugMode = 0;

//Pins map
const int sensorDHT22Pin = 2;
const int startButtonPin = 3;
const int optionButtonPin = 4;
const int cancelButtonPin = 5;
const int frioPin = 6;
const int calorPin = 7;
const int deshumidificarPin = 8;
const int humidificarPin = 9;
//Constantes
const int readingFrecuencyTime = 5000;
const int estufadoTemperatura = 24;
const int estufadoHumedad = 94;
const int preSecadoTemperatura = 18;
const int preSecadoHumedad = 84;
const int secadoTemperatura = 14;
const int secadoHumedad = 72;
const int deltaT = 1;
const int deltaH = 4;

float currentTemperature;
float currentHumidity;

String programs[6];
int selectedProgram = 99;
int indexOfPrograms = 6;
int pins[14];
int firstLecture = 1;
int mustPrintTemperaturaAndState;

unsigned int lastPrintedSeconds = -1;
String lastPrintedText;
String lastPrintedSubprogram;

Bounce2::Button startButton = Bounce2::Button();
Bounce2::Button optionButton = Bounce2::Button();
Bounce2::Button cancelButton = Bounce2::Button();

enum machineStates {idleMachine, estufado, secado, preSecado, actionRunning};
enum charcuteriaProgramStates {idleProgram, programRunning };
enum machineStates currentMachineState;
enum charcuteriaProgramStates currentProgramState;

#define accumulatedMillis millis() - timerMillis

unsigned long timerMillis;

void setup() {
  //Inicialize the machine
  setupMachinePinsMode();
  setupButtons();
  initializeMachineState();
  initializePrograms();
  initializePins();
  //Restaura el programa seleccionado desde la memoria EEPROM
  restoreProgramFromMemory();
  setupLCDInitialState();
  }

void setupMachinePinsMode(){
  pinMode(frioPin, OUTPUT);
  pinMode(calorPin, OUTPUT);
  pinMode(deshumidificarPin, OUTPUT);
  pinMode(humidificarPin, OUTPUT);
  }

void initializeMachineState() {
  digitalWrite(frioPin, HIGH);
  digitalWrite(calorPin, HIGH);
  digitalWrite(deshumidificarPin, HIGH);
  digitalWrite(humidificarPin, HIGH);
  currentMachineState = idleMachine;
  currentProgramState = idleProgram;
  selectedProgram = 99;
  firstLecture = 1;
  mustPrintTemperaturaAndState = 1;
  currentTemperature = 0;
  currentHumidity = 0;
}

void setupButtons() {
  startButton.attach( startButtonPin, INPUT ); // USE EXTERNAL PULL-UP
  optionButton.attach( optionButtonPin, INPUT ); 
  cancelButton.attach( cancelButtonPin, INPUT ); 
  startButton.interval(5); 
  optionButton.interval(5);
  cancelButton.interval(5);
  startButton.setPressedState(LOW); 
  optionButton.setPressedState(LOW); 
  cancelButton.setPressedState(LOW); 
  }

void setupLCDInitialState() {
  // Initiate the LCD:
  initOutput();
  setCursor(0, 0); 
  print("Chachi Charcuteria");
  updateLCDState();
  setCursor(0, 1);
  }

void updateLCDState() {
  if (selectedProgram==99) {
    setCursor(0, 2);
    print("                   ");
    setCursor(0, 2);
    print("Boton amarillo para");
    setCursor(0, 3);
    print("                   ");
    setCursor(0, 3);
    print("seleccionar programa");
   }
  else {
    lcd.clear();
    setCursor(0, 0);
    print("                    ");
    setCursor(0, 0);
    print("P");
    print(String(selectedProgram));
    print(":");
    print(programs[selectedProgram]);
    if ((selectedProgram >= 3) && digitalRead(pins[selectedProgram]) == LOW) {
        printCurrentState();}
    else {
      setCursor(0,2);
      print("Boton blanco para");
      setCursor(0,3);
      print("iniciar!");}
  }
  }

void readHumidityAndTemperature() {
      int chk = DHT.read22(sensorDHT22Pin);
      switch (chk)
     {
        case DHTLIB_OK:
          int nowTemperature = DHT.temperature;
          int nowHumidity = DHT.humidity;
          if (notEquatFloats(currentTemperature, nowTemperature) || notEquatFloats(currentHumidity, nowHumidity)) {
            currentTemperature = nowTemperature;
            currentHumidity = nowHumidity;
            mustPrintTemperaturaAndState = 1;
            }
          break;
        case DHTLIB_ERROR_CHECKSUM: 
          lcd.clear();
          print("Checksum error,\t"); 
          break;
        case DHTLIB_ERROR_TIMEOUT: 
          lcd.clear();
          print("Time out error,\t"); 
          break;
        default: 
          lcd.clear();
          print("Unknown error,\t"); 
          break;
      }
  }

void loop() {
  startButton.update();  
  optionButton.update();
  cancelButton.update();
  if (cancelButton.pressed()) {
    resetMachine();
    }
  switch (currentMachineState) {
    case idleMachine: 
      if (startButton.pressed()) {
        startProgram();
        }
        break;
    case estufado:
        updateProgram(estufadoTemperatura, estufadoHumedad);
        break;
    case preSecado:
         updateProgram(preSecadoTemperatura, preSecadoHumedad);
        break;
    case secado:
        updateProgram(secadoTemperatura, secadoHumedad);
        break;
    case actionRunning:
        if (startButton.pressed()) {
          startOrShutdownActions(selectedProgram);}
        break; 
  }  
  if (optionButton.pressed() && (currentMachineState == idleMachine || currentMachineState == actionRunning))  { 
     selectedProgram = selectedProgram + 1;
     if ((selectedProgram > indexOfPrograms) || (selectedProgram < 0)) {
          selectedProgram = 0;
        }
     updateLCDState();
     //Si es una acción, actualiza el estado de la salida
     if (selectedProgram >= 3) {
        currentMachineState = actionRunning;
        //printStateOf(pins[selectedProgram], 0, 3);  
        }
       else {
        //Si no hay ninguna acción ejecutando deja la maquina en idle
        if ((currentMachineState == actionRunning) && (notActionRunning())) { currentMachineState = idleMachine;}
       }
}
}

void resetMachine(){
  initializeMachineState();
  //Guardo el código del cancelar en la memoria EEPROM
  EEPROM.update(0, 99); 
  setupLCDInitialState();
  }

void updateStateOnSelectedProgram() {
  switch (selectedProgram) {
    case 0:
        currentMachineState = estufado;
        currentProgramState = idleProgram;
        break;
    case 1:
        currentMachineState = preSecado;
        currentProgramState = idleProgram;
        break;        
    case 2:
        currentMachineState = secado;
        currentProgramState = idleProgram;
        break;
    default:
        currentMachineState = actionRunning;
        currentProgramState = idleProgram;
        break;
    }
  }

bool notActionRunning() {
  //Answer true is there aren't any action running
  bool actionRunning;
  actionRunning = false;
  for (int i=3; i<=6; i++) {
    //LOW=0=false is when the action is running
    actionRunning = actionRunning || !digitalRead(pins[i]);
  }
  return !actionRunning;
}

void updateProgram(int t, int h)  {
    //Chequea que haya pasado el tiempo que seteamos en readingFrecuencyTime para volver a leer
    if ((accumulatedMillis >= readingFrecuencyTime) || (firstLecture==1)) {
      firstLecture = 0;      
      readHumidityAndTemperature();
      //Prendo el frio si la temperatura supero la temp del programa + el deltaT
      if ((currentTemperature-t) >= deltaT) { digitalWrite(frioPin, LOW);}
      //Apago el frio si la temperatura es igual o inferior la temp del programa
      if ((currentTemperature) <= t) {digitalWrite(frioPin, HIGH);}
      //Prendo el calor si la temperatura es inferior la temp del programa - el deltaT
      if ((t-currentTemperature) >= deltaT) {digitalWrite(calorPin, LOW);}
      //Apago el calor si la temperatura es igual o superior la temp del programa
      if ((currentTemperature) >= t) {digitalWrite(calorPin, HIGH);}
      //Prendo el deshumifificador (cooler) si la humedad supero la temp del programa + el deltaT
      if ((currentHumidity-h) >= deltaH) {digitalWrite(deshumidificarPin, LOW);}
      //Apago el deshumifificador si la temperatura es igual o inferior la temp del programa
      if ((currentHumidity) <= h) {digitalWrite(deshumidificarPin, HIGH);}
      //Prendo el huidificar si la humedad es inferior la temp del programa - el deltaT
      if ((h-currentHumidity) >= deltaH) {digitalWrite(humidificarPin, LOW);}
      //Apago el calor si la temperatura es igual o superior la temp del programa
      if ((currentHumidity) >= h) {digitalWrite(humidificarPin, HIGH);}
      if (mustPrintTemperaturaAndState==1) {
        printTemperatureAndHumidity();
        printCurrentState();
        mustPrintTemperaturaAndState = 0;} 
      //reseto el contador
      timerMillis = millis();
    }  
 }

void startOrShutdownActions (int program) {
  if (program>=3) {
     digitalWrite(pins[program], !digitalRead(pins[program]));
     setCursor(0,2);
     print("                    ");
     printCurrentState();}
}

void printStateOf (int pin, int col, int row) {
   setCursor(col, row);
   if (digitalRead(pin) == LOW) {
        print("SI");
        }
   else {
        print("NO");
        }    
   }

void initializePrograms() {
  programs[0] = "Estuf.(" + String(estufadoTemperatura) + String(char(223)) + "C/" + String(estufadoHumedad)+ "%H)";
  programs[1] = "PreSe.(" + String(preSecadoTemperatura) + String(char(223)) + "C/" + String(preSecadoHumedad)+ "%H)";
  programs[2] = "Secado(" + String(secadoTemperatura) + String(char(223)) + "C/" + String(secadoHumedad)+ "%H)";
  programs[3] = "Calor";
  programs[4] = "Frio";
  programs[5] = "Deshumidificar";
  programs[6] = "Humidificar";
  }

void initializePins() {
  //Mapeo de Pins y programas
  pins[3] = frioPin;
  pins[4] = calorPin;
  pins[5] = deshumidificarPin;
  pins[6] = humidificarPin;
}

void printCurrentState() {
  setCursor(0,3);
  print("                    ");
  setCursor(0,3);
  print("F=");
  printStateOf(frioPin, 2, 3);
  setCursor(5,3);  
  print("C=");
  printStateOf(calorPin, 7, 3);
  setCursor(10,3);  
  print("H=");
  printStateOf(humidificarPin, 12, 3);
  setCursor(15,3);  
  print("D=");
  printStateOf(deshumidificarPin, 17, 3);
  }

void restoreProgramFromMemory() {
  Serial.println(millis());
  selectedProgram = EEPROM.read(0); 
  if (selectedProgram!=99) {
    startProgram();
  }
  }

void startProgram() {
  timerMillis = millis(); //reset timer
  updateStateOnSelectedProgram();
  //Guardo el programa seleccionado en la memoria EEPROM
  EEPROM.update(0, selectedProgram);
  }

  #ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}


void setCursor(int column, int row) {
  if (debugMode==0) {
    lcd.setCursor(column,row);
  }
  else {
    Serial.println("");    
  }
}

void print(String textToPrint) {
   if (debugMode==0) {
  lcd.print(textToPrint) ;  
   }
  else {
    Serial.print(textToPrint);
  }
}

void print(long longToPrint, int decimal) {
   if (debugMode==0) {
  lcd.print(longToPrint, decimal) ;  
   }
  else {
    Serial.println(longToPrint);
  }
}

void initOutput() {
   if (debugMode==0) {
    lcd.init();
    lcd.backlight(); }
  else {Serial.begin(9600);}
}

void printTemperatureAndHumidity() {
    setCursor(0,1);
    print("Last:");
    print(millis(), 1);
    setCursor(0,2);
    print("T:");
    print(currentTemperature, 1);
    print(String(char(223)));
    print("C    ");
    print("H:");
    print(currentHumidity, 1);
    print("%  "); 
}

boolean notEquatFloats(float float1, float float2) {
  //return ( (int)(float1 *10)) != ((int)(float2 * 10));
  return (round(float1) != round(float2));

}