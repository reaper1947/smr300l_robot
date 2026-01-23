const int relayPin = 1;
const unsigned long timeout = 1000;         // Timeout in milliseconds
const int maxRepeats = 1;                   // Max allowed repeated payloads
unsigned long lastValidTime = 0;            // Timestamp of last valid message
bool relayState = false;
bool lastRelayState = false;

String lastPayload = "";
int repeatCount = 0;

void setup() {
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  Serial.begin(115200);
  Serial.println("XIAO Ready");
}

uint8_t computeChecksum(const String& data) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < data.length(); i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void loop() {
  // Process incoming serial data
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int sep = input.indexOf('|');
    if (sep == -1) {
      Serial.println("ERR: No separator found");
      return;
    }

    String payload = input.substring(0, sep);
    String checksumStr = input.substring(sep + 1);

    uint8_t expected = computeChecksum(payload);
    uint8_t received = (uint8_t) strtol(checksumStr.c_str(), NULL, 10); // Decimal

    if (expected == received) {
      if (payload == lastPayload) {
        repeatCount++;
      } 
      else {
        repeatCount = 0;
        lastPayload = payload;
      }

      if (repeatCount > maxRepeats) {
        relayState = false;
        Serial.println("ERR: Payload repeated, relay OFF");
      } 
      else {
        relayState = true;
        lastValidTime = millis();
        Serial.println("ACK: Valid data received");
      }
    } 
    else {
      Serial.println("ERR: Checksum mismatch");
    }
  }

  // Check for timeout independently of data availability
  if (relayState && (millis() - lastValidTime > timeout)) {
    relayState = false;
    Serial.println("INFO: Relay OFF due to timeout");
  }

  // Update relay pin if state changed
  if (relayState != lastRelayState) {
    digitalWrite(relayPin, relayState ? HIGH : LOW);
    Serial.println(relayState ? "OK: Relay ON" : "INFO: Relay OFF");
    lastRelayState = relayState;
  }
}
