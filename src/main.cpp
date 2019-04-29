//***************************************************************************
// Base project: https://bogza.ro/index.php/Smart_presence_sensor_T19SMWV01
// Github: https://github.com/tinel-c/HomieTest
//***************************************************************************
#include <Homie.h>

// set up the time to send the temperature
const int DEFAULT_TEMPERATURE_INTERVAL = 30;
//construct a global variable to handle the temperature
unsigned long lastTemperatureSent = 0;
// create a temperature node
HomieNode ComplexSensorNode("complexSensor", "ComplexSensor","string");
HomieSetting<long> temperatureIntervalSetting("temperatureInterval", "The temperature interval in seconds");

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
}

void loop() {
  Homie.loop();
}