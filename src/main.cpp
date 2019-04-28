//***************************************************************************
// Base project: https://bogza.ro/index.php/Smart_presence_sensor_T19SMWV01
// Github: https://github.com/tinel-c/HomieTest
//***************************************************************************
#include <Homie.h>

HomieNode lightNode("light", "Light", "switch");

bool globalInputHandler(const HomieNode& node, const HomieRange& range, const String& property, const String& value) {
  Homie.getLogger() << "Received on node " << node.getId() << ": " << property << " = " << value << endl;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial << endl << endl;
  Homie_setFirmware("TinelIotBaseSoftware", "1.0.1");
  Homie.setGlobalInputHandler(globalInputHandler);

  lightNode.advertise("on").setName("On").setDatatype("boolean").settable();

  // set the name of the device family
  Homie_setBrand("T19SMWV01");
  // stup homie
  Homie.setup();
}

void loop() {
  Homie.loop();
}