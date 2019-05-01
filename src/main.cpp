//***************************************************************************
// Base project: https://bogza.ro/index.php/Smart_presence_sensor_T19SMWV01
// Github: https://github.com/tinel-c/HomieTest
//***************************************************************************
#include <Homie.h>
//https://github.com/Martinsos/arduino-lib-hc-sr04
#include <HCSR04.h>

// Trig from HCSR04
const byte trigHCSR04 = 4;
// Echo from HCSR04
const byte echoHCSR04 = 5;
// interrupt configurarion PIR
const byte interruptPinPIR = 14;
// interrupt configurarion RADAR
const byte interruptPinRadar = 2;

unsigned long systemTime = -1; // get the system time
volatile byte pirPinChanges = 0; // count the number of chages in the PIR sensor
volatile byte radarPinChanges = 0; // count the number of chages in the Radar sensor

// configure pins for the ultrasonic sensor
UltraSonicDistanceSensor distanceSensor(trigHCSR04, echoHCSR04);  // Initialize sensor that uses digital pins 4 and 5.

// set up the time to send the temperature
const int DEFAULT_TEMPERATURE_INTERVAL = 30;
//construct a global variable to handle the temperature
unsigned long lastTemperatureSent = 0;
// create a temperature node
HomieNode ComplexSensorNode("complexSensor", "ComplexSensor","string");
HomieSetting<long> temperatureIntervalSetting("temperatureInterval", "The temperature interval in seconds");


void TaskReadUltrasonicSensor() {
  // Every 1000 miliseconds, do a measurement using the sensor and print the distance in centimeters.
  Serial.print("Distance measured in cm: ");
  Serial.println(distanceSensor.measureDistanceCm());
}

//Interupt PIR
void ICACHE_RAM_ATTR handleInterruptPIR() {
  pirPinChanges = pirPinChanges + 1;
}

//Interupt Radar
void ICACHE_RAM_ATTR handleInterruptRadar() {
  radarPinChanges = radarPinChanges + 1;
}


bool globalInputHandler(const HomieNode& node, const HomieRange& range, const String& property, const String& value) {
  Homie.getLogger() << "Received on node " << node.getId() << ": " << property << " = " << value << endl;
  return true;
}

void setupHandler() {
  
}

void loopHandler() {
  static float temperature = 22; // Fake temperature here, for the example
  if (millis() - lastTemperatureSent >= temperatureIntervalSetting.get() * 1000UL || lastTemperatureSent == 0) {
    temperature = temperature + 0.1;
    Homie.getLogger() << "Temperature: " << temperature << " °C" << endl;
    ComplexSensorNode.setProperty("tempValue").send(String(temperature));
    ComplexSensorNode.setProperty("number").send(String(temperature*100));
    lastTemperatureSent = millis();
  }
}

void setup() {
  // start the serial communication
  Serial.begin(115200);
  Serial << endl << endl;

  // set the name of the device family
  Homie_setBrand("T19SMWV01");
  //set the name and version of the sensor
  Homie_setFirmware("TinelIotBaseSoftware", "1.0.1");
  // launch the global input handler
  Homie.setGlobalInputHandler(globalInputHandler);
  // start advertise
  ComplexSensorNode.advertise("on").setName("On").setDatatype("boolean").settable();
  ComplexSensorNode.advertise("tempValue").setName("TempValue").setDatatype("string").setUnit("°C");
  ComplexSensorNode.advertise("number").setName("DimmerChannel").setDatatype("integer").settable();
  
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);

  temperatureIntervalSetting.setDefaultValue(DEFAULT_TEMPERATURE_INTERVAL).setValidator([] (long candidate) {
    return candidate > 0;
  });

  // stup homie
  Homie.setup();

  // configure interrupt for PIR
  pinMode(interruptPinPIR, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinPIR), handleInterruptPIR, CHANGE);
  // configure interrupt for PIR
  pinMode(interruptPinRadar, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinRadar), handleInterruptRadar, CHANGE);
}

void loop() {
  Homie.loop();
  if((systemTime < 0) || (millis() - systemTime >=2000))
  {
    systemTime = millis();
    TaskReadUltrasonicSensor();
    Serial.print("Time since the system started: ");
    Serial.println(systemTime); //prints time since program started
    if(pirPinChanges > 0)
    {
      Serial.print("PIR status changed: ");
      Serial.println(digitalRead(interruptPinPIR));      
      pirPinChanges = 0;
    }
    if(radarPinChanges > 0)
    {
      Serial.print("Radar status changed: ");
      Serial.println(digitalRead(interruptPinRadar));
      radarPinChanges = 0;
    }
  }
}