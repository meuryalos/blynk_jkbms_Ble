
#define BLYNK_PRINT Serial
#define BLYNK_FIRMWARE_VERSION "1.0.1"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
WiFiUDP ntpUDP;

NTPClient timeClient(ntpUDP, "time.google.com");
#define DEBUG_ENABLED true

#if DEBUG_ENABLED
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTF(...)
#endif
String formattedTime;
  int x=0;


// WiFi credentials
const char* ssid = "tolong di isi";
const char* pass = "tolong di isi";
char auth[] = "tolong di isi";
WebServer server(80);  // Webserver auf Port 80

uint32_t minFreeHeap = UINT32_MAX;
unsigned long lastHeapUpdate = 0;

unsigned long lastMillis = 0;
uint32_t totalSeconds = 0;
void calculateUptime() {
  unsigned long currentMillis = millis();
  unsigned long elapsedMillis = currentMillis - lastMillis;

  // Überlauf von millis() behandeln
  if (currentMillis < lastMillis) {
    // Ein Überlauf ist aufgetreten
    elapsedMillis = UINT32_MAX - lastMillis + currentMillis;
  }

  
 if (elapsedMillis >= 1000) {
    // Vergangene Sekunden addieren
    totalSeconds += elapsedMillis / 1000;

    // Aktuellen millis()-Wert speichern
    lastMillis = currentMillis;
  }
}

void monitorFreeHeap() {
  uint32_t currentFreeHeap = ESP.getFreeHeap();

  if (currentFreeHeap < minFreeHeap) {
    minFreeHeap = currentFreeHeap;
  }

  if (millis() - lastHeapUpdate >= 1000) {
    minFreeHeap = UINT32_MAX;
    lastHeapUpdate = millis();
  }
}


// lesbare Anzeige der Speichergrößen
void formatBytes(size_t bytes, char* buffer, size_t bufferSize) {
  if (bytes < 1024) {
    snprintf_P(buffer, bufferSize, PSTR("%zu%s"), bytes, PSTR(" Byte"));
  } else if (bytes < 1048576) {
    dtostrf(static_cast<float>(bytes) / 1024.0, 6, 2, buffer);
    strcat_P(buffer, PSTR(" KB"));
  } else {
    dtostrf(static_cast<float>(bytes) / 1048576.0, 6, 2, buffer);
    strcat_P(buffer, PSTR(" MB"));
  }
}


//********************************************
// JKBMS Class Definition
//********************************************
class JKBMS {
public:
  JKBMS(const std::string& mac)
    : targetMAC(mac) {}

  // BLE Components
  NimBLERemoteCharacteristic* pChr = nullptr;
  const NimBLEAdvertisedDevice* advDevice = nullptr;
  bool doConnect = false;
  bool connected = false;
  uint32_t lastNotifyTime = 0;
  std::string targetMAC;

  // Data Processing
  byte receivedBytes[320];
  int frame = 0;
  bool received_start = false;
  bool received_complete = false;
  bool new_data = false;
  int ignoreNotifyCount = 0;

  // BMS Data Fields
  float cellVoltage[16] = { 0 };
  float wireResist[16] = { 0 };
  float Average_Cell_Voltage = 0;
  float Delta_Cell_Voltage = 0;
  float Battery_Voltage = 0;
  float Battery_Power = 0;
  float Charge_Current = 0;
  float Battery_T1 = 0;
  float Battery_T2 = 0;
  float MOS_Temp = 0;
  int Percent_Remain = 0;
  float Capacity_Remain = 0;
  float Nominal_Capacity = 0;
  float Cycle_Count = 0;
  float Cycle_Capacity = 0;
  uint32_t Uptime;
  uint8_t sec, mi, hr, days;
  float Balance_Curr = 0;
  bool Balance = false;
  bool Charge = false;
  bool Discharge = false;
  int Balancing_Action = 0;

  float balance_trigger_voltage = 0;
  float cell_voltage_undervoltage_protection = 0;
  float cell_voltage_undervoltage_recovery = 0;
  float cell_voltage_overvoltage_protection = 0;
  float cell_voltage_overvoltage_recovery = 0;
  float power_off_voltage = 0;
  float max_charge_current = 0;
  float charge_overcurrent_protection_delay = 0;
  float charge_overcurrent_protection_recovery_time = 0;
  float max_discharge_current = 0;
  float discharge_overcurrent_protection_delay = 0;
  float discharge_overcurrent_protection_recovery_time = 0;
  float short_circuit_protection_recovery_time = 0;
  float max_balance_current = 0;
  float charge_overtemperature_protection = 0;
  float charge_overtemperature_protection_recovery = 0;
  float discharge_overtemperature_protection = 0;
  float discharge_overtemperature_protection_recovery = 0;
  float charge_undertemperature_protection = 0;
  float charge_undertemperature_protection_recovery = 0;
  float power_tube_overtemperature_protection = 0;
  float power_tube_overtemperature_protection_recovery = 0;
  int cell_count = 0;
  float total_battery_capacity = 0;
  float short_circuit_protection_delay = 0;
  float balance_starting_voltage = 0;




  // Methods
  bool connectToServer();
  void parseDeviceInfo();
  void parseData();
  void bms_settings();
  void writeRegister(uint8_t address, uint32_t value, uint8_t length);
  void handleNotification(uint8_t* pData, size_t length);

private:
  uint8_t crc(const uint8_t data[], uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) crc += data[i];
    return crc;
  }
};

//********************************************
// Global Variables and Callbacks
//********************************************
JKBMS jkBmsDevices[] = {
  JKBMS("tolong di isi")   // Only one device configured
};

const int bmsDeviceCount = sizeof(jkBmsDevices) / sizeof(jkBmsDevices[0]);  // Dynamic device count

NimBLEScan* pScan;
unsigned long lastScanTime = 0;

class ClientCallbacks : public NimBLEClientCallbacks {
  JKBMS* bms;
public:
  ClientCallbacks(JKBMS* bmsInstance)
    : bms(bmsInstance) {}

  void onConnect(NimBLEClient* pClient) {
    DEBUG_PRINTF("Connected to %s\n", bms->targetMAC.c_str());
    bms->connected = true;
  }

  void onDisconnect(NimBLEClient* pClient, int reason) {
    DEBUG_PRINTF("%s disconnected, reason: %d\n", bms->targetMAC.c_str(), reason);
    bms->connected = false;
    bms->doConnect = false;
  }
};

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) {
    DEBUG_PRINTF("BLE Device found: %s\n", advertisedDevice->toString().c_str());
    Blynk.virtualWrite(V4, "add", x, String(advertisedDevice->getAddress().toString().c_str()), String(advertisedDevice->getName().c_str()));

    x++;
    for (int i = 0; i < bmsDeviceCount; i++) {
      if (jkBmsDevices[i].targetMAC.empty()) continue;  // Skip empty MAC addresses
      if (advertisedDevice->getAddress().toString() == jkBmsDevices[i].targetMAC && !jkBmsDevices[i].connected && !jkBmsDevices[i].doConnect) {
        DEBUG_PRINTF("Found target device: %s\n", jkBmsDevices[i].targetMAC.c_str());
        jkBmsDevices[i].advDevice = advertisedDevice;
        jkBmsDevices[i].doConnect = true;
        NimBLEDevice::getScan()->stop();
      }
    }
  }
} scanCallbacks;

void notifyCB(NimBLERemoteCharacteristic* pChr, uint8_t* pData, size_t length, bool isNotify) {
  DEBUG_PRINTLN("Notification received...");
  for (int i = 0; i < bmsDeviceCount; i++) {
    if (jkBmsDevices[i].pChr == pChr) {
      jkBmsDevices[i].handleNotification(pData, length);
      break;
    }
  }
}

//********************************************
// JKBMS Method Implementation
//********************************************
bool JKBMS::connectToServer() {
  DEBUG_PRINTF("Attempting to connect to %s...\n", targetMAC.c_str());
  NimBLEClient* pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());

  if (!pClient) {
    pClient = NimBLEDevice::createClient();
    DEBUG_PRINTLN("New client created.");
    pClient->setClientCallbacks(new ClientCallbacks(this), true);
    pClient->setConnectionParams(12, 12, 0, 150);
    pClient->setConnectTimeout(5000);
  }

  if (!pClient->connect(advDevice)) {
    DEBUG_PRINTF("Failed to connect to %s\n", targetMAC.c_str());
    return false;
  }

  DEBUG_PRINTF("Connected to: %s RSSI: %d\n", pClient->getPeerAddress().toString().c_str(), pClient->getRssi());

  NimBLERemoteService* pSvc = pClient->getService("ffe0");
  if (pSvc) {
    pChr = pSvc->getCharacteristic("ffe1");
    if (pChr && pChr->canNotify()) {
      if (pChr->subscribe(true, notifyCB)) {
        DEBUG_PRINTF("Subscribed to notifications for %s\n", pChr->getUUID().toString().c_str());
        delay(500);
        writeRegister(0x97, 0x00000000, 0x00);  // COMMAND_DEVICE_INFO
        delay(500);
        writeRegister(0x96, 0x00000000, 0x00);  // COMMAND_CELL_INFO
        return true;
      }
    }
  }
  DEBUG_PRINTLN("Service or Characteristic not found or unable to subscribe.");
  return false;
}

void JKBMS::handleNotification(uint8_t* pData, size_t length) {
  DEBUG_PRINTLN("Handling notification...");
  lastNotifyTime = millis();

  if (ignoreNotifyCount > 0) {
    ignoreNotifyCount--;
    DEBUG_PRINTF("Ignoring notification. Remaining: %d\n", ignoreNotifyCount);
    return;
  }

  // Check for start of data frame
  if (pData[0] == 0x55 && pData[1] == 0xAA && pData[2] == 0xEB && pData[3] == 0x90) {
    DEBUG_PRINTLN("Start of data frame detected.");
    frame = 0;
    received_start = true;
    received_complete = false;

    // Store the received data
    for (int i = 0; i < length; i++) {
      receivedBytes[frame++] = pData[i];
    }
  } else if (received_start && !received_complete) {
    DEBUG_PRINTLN("Continuing data frame...");
    for (int i = 0; i < length; i++) {
      receivedBytes[frame++] = pData[i];
      if (frame >= 300) {
        received_complete = true;
        received_start = false;
        new_data = true;
        DEBUG_PRINTLN("New data available for parsing.");

        // Determine the type of data frame based on pData[4]
        switch (receivedBytes[4]) {  // Use receivedBytes[4] instead of pData[4]
          case 0x01:
            DEBUG_PRINTLN("BMS Settings frame detected.");
            bms_settings();
            break;
          case 0x02:
            DEBUG_PRINTLN("Cell data frame detected.");
            parseData();
            break;
          case 0x03:
            DEBUG_PRINTLN("Device info frame detected.");
            parseDeviceInfo();
            break;
          default:
            DEBUG_PRINTF("Unknown frame type: 0x%02X\n", receivedBytes[4]);
            break;
        }

        break;  // Exit the loop after processing the complete frame
      }
    }
  }
}

void JKBMS::writeRegister(uint8_t address, uint32_t value, uint8_t length) {
  DEBUG_PRINTF("Writing register: address=0x%02X, value=0x%08lX, length=%d\n", address, value, length);
  uint8_t frame[20] = { 0xAA, 0x55, 0x90, 0xEB, address, length };

  // Insert value (Little-Endian)
  frame[6] = value >> 0;  // LSB
  frame[7] = value >> 8;
  frame[8] = value >> 16;
  frame[9] = value >> 24;  // MSB

  // Calculate CRC
  frame[19] = crc(frame, 19);

  // Debug: Print the entire frame in hexadecimal format
  DEBUG_PRINTF("Frame to be sent: ");
  for (int i = 0; i < sizeof(frame); i++) {
    DEBUG_PRINTF("%02X ", frame[i]);
  }
  DEBUG_PRINTF("\n");

  if (pChr) {
    pChr->writeValue((uint8_t*)frame, (size_t)sizeof(frame));
  }
}

void JKBMS::bms_settings() {
  DEBUG_PRINTLN("Processing BMS settings...");
  cell_voltage_undervoltage_protection = ((receivedBytes[13] << 24 | receivedBytes[12] << 16 | receivedBytes[11] << 8 | receivedBytes[10]) * 0.001);
  cell_voltage_undervoltage_recovery = ((receivedBytes[17] << 24 | receivedBytes[16] << 16 | receivedBytes[15] << 8 | receivedBytes[14]) * 0.001);
  cell_voltage_overvoltage_protection = ((receivedBytes[21] << 24 | receivedBytes[20] << 16 | receivedBytes[19] << 8 | receivedBytes[18]) * 0.001);
  cell_voltage_overvoltage_recovery = ((receivedBytes[25] << 24 | receivedBytes[24] << 16 | receivedBytes[23] << 8 | receivedBytes[22]) * 0.001);
  balance_trigger_voltage = ((receivedBytes[29] << 24 | receivedBytes[28] << 16 | receivedBytes[27] << 8 | receivedBytes[26]) * 0.001);
  power_off_voltage = ((receivedBytes[49] << 24 | receivedBytes[48] << 16 | receivedBytes[47] << 8 | receivedBytes[46]) * 0.001);
  max_charge_current = ((receivedBytes[53] << 24 | receivedBytes[52] << 16 | receivedBytes[51] << 8 | receivedBytes[50]) * 0.001);
  charge_overcurrent_protection_delay = ((receivedBytes[57] << 24 | receivedBytes[56] << 16 | receivedBytes[55] << 8 | receivedBytes[54]));
  charge_overcurrent_protection_recovery_time = ((receivedBytes[61] << 24 | receivedBytes[60] << 16 | receivedBytes[59] << 8 | receivedBytes[58]));
  max_discharge_current = ((receivedBytes[65] << 24 | receivedBytes[64] << 16 | receivedBytes[63] << 8 | receivedBytes[62]) * 0.001);
  discharge_overcurrent_protection_delay = ((receivedBytes[69] << 24 | receivedBytes[68] << 16 | receivedBytes[67] << 8 | receivedBytes[66]));
  discharge_overcurrent_protection_recovery_time = ((receivedBytes[73] << 24 | receivedBytes[72] << 16 | receivedBytes[71] << 8 | receivedBytes[70]));
  short_circuit_protection_recovery_time = ((receivedBytes[77] << 24 | receivedBytes[76] << 16 | receivedBytes[75] << 8 | receivedBytes[74]));
  max_balance_current = ((receivedBytes[81] << 24 | receivedBytes[80] << 16 | receivedBytes[79] << 8 | receivedBytes[78]) * 0.001);
  charge_overtemperature_protection = ((receivedBytes[85] << 24 | receivedBytes[84] << 16 | receivedBytes[83] << 8 | receivedBytes[82]) * 0.1);
  charge_overtemperature_protection_recovery = ((receivedBytes[89] << 24 | receivedBytes[88] << 16 | receivedBytes[87] << 8 | receivedBytes[86]) * 0.1);
  discharge_overtemperature_protection = ((receivedBytes[93] << 24 | receivedBytes[92] << 16 | receivedBytes[91] << 8 | receivedBytes[90]) * 0.1);
  discharge_overtemperature_protection_recovery = ((receivedBytes[97] << 24 | receivedBytes[96] << 16 | receivedBytes[95] << 8 | receivedBytes[94]) * 0.1);
  charge_undertemperature_protection = ((receivedBytes[101] << 24 | receivedBytes[100] << 16 | receivedBytes[99] << 8 | receivedBytes[98]) * 0.1);
  charge_undertemperature_protection_recovery = ((receivedBytes[105] << 24 | receivedBytes[104] << 16 | receivedBytes[103] << 8 | receivedBytes[102]) * 0.1);
  power_tube_overtemperature_protection = ((receivedBytes[109] << 24 | receivedBytes[108] << 16 | receivedBytes[107] << 8 | receivedBytes[106]) * 0.1);
  power_tube_overtemperature_protection_recovery = ((receivedBytes[113] << 24 | receivedBytes[112] << 16 | receivedBytes[111] << 8 | receivedBytes[110]) * 0.1);
  cell_count = ((receivedBytes[117] << 24 | receivedBytes[116] << 16 | receivedBytes[115] << 8 | receivedBytes[114]));
  // 118   4   0x01 0x00 0x00 0x00    Charge switch
  // 122   4   0x01 0x00 0x00 0x00    Discharge switch
  // 126   4   0x01 0x00 0x00 0x00    Balancer switch
  total_battery_capacity = ((receivedBytes[133] << 24 | receivedBytes[132] << 16 | receivedBytes[131] << 8 | receivedBytes[130]) * 0.001);
  short_circuit_protection_delay = ((receivedBytes[137] << 24 | receivedBytes[136] << 16 | receivedBytes[135] << 8 | receivedBytes[134]) * 1);
  balance_starting_voltage = ((receivedBytes[141] << 24 | receivedBytes[140] << 16 | receivedBytes[139] << 8 | receivedBytes[138]) * 0.001);




  DEBUG_PRINTF("Cell voltage undervoltage protection: %.2fV\n", cell_voltage_undervoltage_protection);
  DEBUG_PRINTF("Cell voltage undervoltage recovery: %.2fV\n", cell_voltage_undervoltage_recovery);
  DEBUG_PRINTF("Cell voltage overvoltage protection: %.2fV\n", cell_voltage_overvoltage_protection);
  DEBUG_PRINTF("Cell voltage overvoltage recovery: %.2fV\n", cell_voltage_overvoltage_recovery);
  DEBUG_PRINTF("Balance trigger voltage: %.2fV\n", balance_trigger_voltage);
  DEBUG_PRINTF("Power off voltage: %.2fV\n", power_off_voltage);

  DEBUG_PRINTF("Max charge current: %.2fA\n", max_charge_current);
  DEBUG_PRINTF("Charge overcurrent protection delay: %.2fs\n", charge_overcurrent_protection_delay);
  DEBUG_PRINTF("Charge overcurrent protection recovery time: %.2fs\n", charge_overcurrent_protection_recovery_time);
  DEBUG_PRINTF("Max discharge current: %.2fA\n", max_discharge_current);
  DEBUG_PRINTF("Discharge overcurrent protection delay: %.2fs\n", discharge_overcurrent_protection_delay);
  DEBUG_PRINTF("Discharge overcurrent protection recovery time: %.2fs\n", discharge_overcurrent_protection_recovery_time);
  DEBUG_PRINTF("Short circuit protection recovery time: %.2fs\n", short_circuit_protection_recovery_time);
  DEBUG_PRINTF("Max balance current: %.2fA\n", max_balance_current);
  DEBUG_PRINTF("Charge overtemperature protection: %.2fC\n", charge_overtemperature_protection);
  DEBUG_PRINTF("Charge overtemperature protection recovery: %.2fC\n", charge_overtemperature_protection_recovery);
  DEBUG_PRINTF("Discharge overtemperature protection: %.2fC\n", discharge_overtemperature_protection);
  DEBUG_PRINTF("Discharge overtemperature protection recovery: %.2fC\n", discharge_overtemperature_protection_recovery);
  DEBUG_PRINTF("Charge undertemperature protection: %.2fC\n", charge_undertemperature_protection);
  DEBUG_PRINTF("Charge undertemperature protection recovery: %.2fC\n", charge_undertemperature_protection_recovery);
  DEBUG_PRINTF("Power tube overtemperature protection: %.2fC\n", power_tube_overtemperature_protection);
  DEBUG_PRINTF("Power tube overtemperature protection recovery: %.2fC\n", power_tube_overtemperature_protection_recovery);
  DEBUG_PRINTF("Cell count: %.d\n", cell_count);
  DEBUG_PRINTF("Total battery capacity: %.2fAh\n", total_battery_capacity);
  DEBUG_PRINTF("Short circuit protection delay: %.2fus\n", short_circuit_protection_delay);
  DEBUG_PRINTF("Balance starting voltage: %.2fV\n", balance_starting_voltage);
}

void JKBMS::parseDeviceInfo() {
  DEBUG_PRINTLN("Processing device info...");
  new_data = false;

  // Debugging: Ausgabe der empfangenen Bytes
  DEBUG_PRINTLN("Raw data received:");
  for (int i = 0; i < frame; i++) {
    DEBUG_PRINTF("%02X ", receivedBytes[i]);
    if ((i + 1) % 16 == 0) DEBUG_PRINTLN();  // Neue Zeile nach 16 Bytes
  }
  DEBUG_PRINTLN();

  // Überprüfen, ob genügend Daten empfangen wurden
  if (frame < 134) {  // 134 Bytes sind für die Geräteinformationen erforderlich
    DEBUG_PRINTLN("Error: Not enough data received for device info.");
    return;
  }

  // Extrahieren der Geräteinformationen aus den empfangenen Bytes
  std::string vendorID(receivedBytes + 6, receivedBytes + 6 + 16);
  std::string hardwareVersion(receivedBytes + 22, receivedBytes + 22 + 8);
  std::string softwareVersion(receivedBytes + 30, receivedBytes + 30 + 8);
  uint32_t uptime = (receivedBytes[41] << 24) | (receivedBytes[40] << 16) | (receivedBytes[39] << 8) | receivedBytes[38];
  uint32_t powerOnCount = (receivedBytes[45] << 24) | (receivedBytes[44] << 16) | (receivedBytes[43] << 8) | receivedBytes[42];
  std::string deviceName(receivedBytes + 46, receivedBytes + 46 + 16);
  std::string devicePasscode(receivedBytes + 62, receivedBytes + 62 + 16);
  std::string manufacturingDate(receivedBytes + 78, receivedBytes + 78 + 8);
  std::string serialNumber(receivedBytes + 86, receivedBytes + 86 + 11);
  std::string passcode(receivedBytes + 97, receivedBytes + 97 + 5);
  std::string userData(receivedBytes + 102, receivedBytes + 102 + 16);
  std::string setupPasscode(receivedBytes + 118, receivedBytes + 118 + 16);

  // Ausgabe der Geräteinformationen
  DEBUG_PRINTF("  Vendor ID: %s\n", vendorID.c_str());
  DEBUG_PRINTF("  Hardware version: %s\n", hardwareVersion.c_str());
  DEBUG_PRINTF("  Software version: %s\n", softwareVersion.c_str());
  DEBUG_PRINTF("  Uptime: %d s\n", uptime);
  DEBUG_PRINTF("  Power on count: %d\n", powerOnCount);
  DEBUG_PRINTF("  Device name: %s\n", deviceName.c_str());
  DEBUG_PRINTF("  Device passcode: %s\n", devicePasscode.c_str());
  DEBUG_PRINTF("  Manufacturing date: %s\n", manufacturingDate.c_str());
  DEBUG_PRINTF("  Serial number: %s\n", serialNumber.c_str());
  DEBUG_PRINTF("  Passcode: %s\n", passcode.c_str());
  DEBUG_PRINTF("  User data: %s\n", userData.c_str());
  DEBUG_PRINTF("  Setup passcode: %s\n", setupPasscode.c_str());
}

void JKBMS::parseData() {
  DEBUG_PRINTLN("Parsing data...");
  new_data = false;
  ignoreNotifyCount = 10;
  // Cell voltages
  for (int j = 0, i = 7; i < 38; j++, i += 2) {
    cellVoltage[j] = ((receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001);
  }

  Average_Cell_Voltage = (((int)receivedBytes[75] << 8 | receivedBytes[74]) * 0.001);

  Delta_Cell_Voltage = (((int)receivedBytes[77] << 8 | receivedBytes[76]) * 0.001);
  for (int j = 0, i = 81; i < 112; j++, i += 2) {
    wireResist[j] = (((int)receivedBytes[i] << 8 | receivedBytes[i - 1]) * 0.001);
    
  }

  if (receivedBytes[145] == 0xFF) {
    MOS_Temp = ((0xFF << 24 | 0xFF << 16 | receivedBytes[145] << 8 | receivedBytes[144]) * 0.1);
  } else {
    MOS_Temp = ((receivedBytes[145] << 8 | receivedBytes[144]) * 0.1);
  }

  // Battery voltage
  Battery_Voltage = ((receivedBytes[153] << 24 | receivedBytes[152] << 16 | receivedBytes[151] << 8 | receivedBytes[150]) * 0.001);

  Charge_Current = ((receivedBytes[161] << 24 | receivedBytes[160] << 16 | receivedBytes[159] << 8 | receivedBytes[158]) * 0.001);

  Battery_Power = Battery_Voltage * Charge_Current;

  if (receivedBytes[163] == 0xFF) {
    Battery_T1 = ((0xFF << 24 | 0xFF << 16 | receivedBytes[163] << 8 | receivedBytes[162]) * 0.1);
  } else {
    Battery_T1 = ((receivedBytes[163] << 8 | receivedBytes[162]) * 0.1);
  }

  if (receivedBytes[165] == 0xFF) {
    Battery_T2 = ((0xFF << 24 | 0xFF << 16 | receivedBytes[165] << 8 | receivedBytes[164]) * 0.1);
  } else {
    Battery_T2 = ((receivedBytes[165] << 8 | receivedBytes[164]) * 0.1);
  }

  if ((receivedBytes[171] & 0xF0) == 0x0) {
    Balance_Curr = ((receivedBytes[171] << 8 | receivedBytes[170]) * 0.001);
  } else if ((receivedBytes[171] & 0xF0) == 0xF0) {
    Balance_Curr = (((receivedBytes[171] & 0x0F) << 8 | receivedBytes[170]) * -0.001);
  }

  Balancing_Action = receivedBytes[172];
  Percent_Remain = (receivedBytes[173]);
  Capacity_Remain = ((receivedBytes[177] << 24 | receivedBytes[176] << 16 | receivedBytes[175] << 8 | receivedBytes[174]) * 0.001);
  Nominal_Capacity = ((receivedBytes[181] << 24 | receivedBytes[180] << 16 | receivedBytes[179] << 8 | receivedBytes[178]) * 0.001);
  Cycle_Count = ((receivedBytes[185] << 24 | receivedBytes[184] << 16 | receivedBytes[183] << 8 | receivedBytes[182]));
  Cycle_Capacity = ((receivedBytes[189] << 24 | receivedBytes[188] << 16 | receivedBytes[187] << 8 | receivedBytes[186]) * 0.001);

  Uptime = receivedBytes[196] << 16 | receivedBytes[195] << 8 | receivedBytes[194];
  sec = Uptime % 60;
  Uptime /= 60;
  mi = Uptime % 60;
  Uptime /= 60;
  hr = Uptime % 24;
  days = Uptime / 24;

  if (receivedBytes[198] > 0) {
    Charge = true;
  } else if (receivedBytes[198] == 0) {
    Charge = false;
  }
  if (receivedBytes[199] > 0) {
    Discharge = true;
  } else if (receivedBytes[199] == 0) {
    Discharge = false;
  }
  if (receivedBytes[201] > 0) {
    Balance = true;
  } else if (receivedBytes[201] == 0) {
    Balance = false;
  }


  // Output values

  DEBUG_PRINTF("\n--- Data from %s ---\n", targetMAC.c_str());
  DEBUG_PRINTLN("Cell Voltages:");
  for (int j = 0; j < 16; j++) {
    DEBUG_PRINTF("  Cell %02d: %.3f V\n", j + 1, cellVoltage[j]);
  }
  DEBUG_PRINTLN("wire Resist:");
  for (int j = 0; j < 16; j++) {
    DEBUG_PRINTF("  Cell %02d: %.3f Ohm\n", j + 1, wireResist[j]);
  }
  DEBUG_PRINTF("Average Cell Voltage: %.2fV\n", Average_Cell_Voltage);
  DEBUG_PRINTF("Delta Cell Voltage: %.2fV\n", Delta_Cell_Voltage);
  DEBUG_PRINTF("Balance Curr: %.2fA\n", Balance_Curr);
  DEBUG_PRINTF("Battery Voltage: %.2fV\n", Battery_Voltage);
  DEBUG_PRINTF("Battery Power: %.2fW\n", Battery_Power);
  DEBUG_PRINTF("Charge Current: %.2fA\n", Charge_Current);
  DEBUG_PRINTF("Percent_Remain: %d%%\n", Percent_Remain);
  DEBUG_PRINTF("Capacity Remain: %.2fAh\n", Capacity_Remain);
  DEBUG_PRINTF("Nominal Capacity: %.2fAh\n", Nominal_Capacity);
  DEBUG_PRINTF("Cycle Count: %.2f\n", Cycle_Count);
  DEBUG_PRINTF("Cycle Capacity: %.2fAh\n", Cycle_Capacity);
  DEBUG_PRINTF("Temperature T1: %.1fC\n", Battery_T1);
  DEBUG_PRINTF("Temperature T2: %.1fC\n", Battery_T2);
  DEBUG_PRINTF("Temperature MOS: %.1fC\n", MOS_Temp);
  DEBUG_PRINTF("Uptime: %dd %dh %dm\n", days, hr, mi);
  DEBUG_PRINTF("Charge: %d\n", Charge);
  DEBUG_PRINTF("Discharge: %d\n", Discharge);
  DEBUG_PRINTF("Balance: %d\n", Balance);
  DEBUG_PRINTF("Balancing Action: %d\n", Balancing_Action);
  
}
//********************************************
// Main Program
//********************************************
void setup() {
  Serial.begin(115200);
  lastMillis = millis();

  DEBUG_PRINTLN("Initializing NimBLE Client...");

  NimBLEDevice::init("MultiJKBMS-Client");
  NimBLEDevice::setPower(3);

  pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacks);
  pScan->setInterval(100);
  pScan->setWindow(100);
  pScan->setActiveScan(true);
  Blynk.begin(auth, ssid, pass, "tolong di isi", 8080);
   while (Blynk.connect() == false) {}
  timeClient.begin();
  Blynk.virtualWrite(V5, String(WiFi.localIP().toString().c_str()));
  data();
  
}

void loop() {
  Blynk.run();
  timeClient.setTimeOffset(25200);
  timeClient.update();
  formattedTime = timeClient.getFormattedTime();
  if (!Blynk.connected()) {
    Serial.println("Lost connection");
    if (Blynk.connect()) {
      
      Serial.println("Reconnected");
    } else {
      Serial.println("Not reconnected");
    }
  }
  calculateUptime();
  monitorFreeHeap();
  server.handleClient();
  // Connection management
  int connectedCount = 0;  // Counter for connected devices

  for (int i = 0; i < bmsDeviceCount; i++) {
    if (jkBmsDevices[i].targetMAC.empty()) continue;  // Skip empty MAC addresses

    if (jkBmsDevices[i].doConnect && !jkBmsDevices[i].connected) {
      if (jkBmsDevices[i].connectToServer()) {
        DEBUG_PRINTF("%s connected successfully\n", jkBmsDevices[i].targetMAC.c_str());
        Blynk.virtualWrite(V1, "connected successfully");
      }
      jkBmsDevices[i].doConnect = false;
    }

    if (jkBmsDevices[i].connected) {
      connectedCount++;  // Increment counter for each connected device
      if (millis() - jkBmsDevices[i].lastNotifyTime > 20000) {
        DEBUG_PRINTF("%s connection timeout\n", jkBmsDevices[i].targetMAC.c_str());
        Blynk.virtualWrite(V1, "connection timeout");
        NimBLEClient* pClient = NimBLEDevice::getClientByPeerAddress(jkBmsDevices[i].advDevice->getAddress());
        if (pClient) pClient->disconnect();
      }
    }
  }

  // Start scan only if not all devices are connected
  if (connectedCount < bmsDeviceCount && (millis() - lastScanTime >= 10000)) {
    DEBUG_PRINTLN("Starting scan...");
    Blynk.virtualWrite(V1, "Starting scan...");
    Blynk.virtualWrite(V4, "clr");
    pScan->start(5000, false, true);
    lastScanTime = millis();
  }
  updatedata();
  delay(10);
}
void data () {
    Blynk.virtualWrite(V66, "clr");
    Blynk.virtualWrite(V10, "clr");
    Blynk.virtualWrite(V11, "clr");
    bridge1.virtualWrite(V66, "clr");
    for (int j = 0; j < 15; j++) {
      Blynk.virtualWrite(V10, "add", j, "cellVoltage " + String(j), String(j));
    }
    for (int j = 0; j < 15; j++) {
      Blynk.virtualWrite(V11, "add", j, "wireResist " + String(j), String(j));
    }
    Blynk.virtualWrite(V66, "add", 0, "Time", String(formattedTime) + " " + "WIB");
    Blynk.virtualWrite(V66, "add", 1, "battery_voltage", "battery_voltage");
    Blynk.virtualWrite(V66, "add", 2, "battery_power","battery_power");
    Blynk.virtualWrite(V66, "add", 3, "Charge_Current","Charge_Current");
    Blynk.virtualWrite(V66, "add", 4, "Percent_Remain","Percent_Remain");
    Blynk.virtualWrite(V66, "add", 5, "capacity_remain","capacity_remain");
    Blynk.virtualWrite(V66, "add", 6, "nominal_capacity","nominal_capacity");
    Blynk.virtualWrite(V66, "add", 7, "cycle_count","cycle_count");
    Blynk.virtualWrite(V66, "add", 8, "cycle_capacity", "cycle_capacity");
    Blynk.virtualWrite(V66, "add", 9, "battery_t1", "battery_t1");
    Blynk.virtualWrite(V66, "add", 10, "battery_t2","battery_t2");
    Blynk.virtualWrite(V66, "add", 11, "mos_temp", "mos_temp");
    Blynk.virtualWrite(V66, "add", 12, "average_cell_voltage", "average_cell_voltage");
    Blynk.virtualWrite(V66, "add", 13, "delta_cell_voltage", "delta_cell_voltage");
    Blynk.virtualWrite(V66, "add", 14, "Balance_Curr", "Balance_Curr");


    
}
void updatedata (){
  if (bmsDeviceCount > 0){
  JKBMS& bms = jkBmsDevices[0];
  Serial.println(bms.targetMAC.c_str());

  Blynk.virtualWrite(V45, String(bms.Battery_Voltage));
  Blynk.virtualWrite(V46, String(bms.Charge_Current));
  Blynk.virtualWrite(V47, String(bms.Battery_Power));
  Blynk.virtualWrite(V48, String(bms.Percent_Remain));
  bridge1.virtualWrite(V45, String(bms.Battery_Voltage));
  bridge1.virtualWrite(V46, String(bms.Charge_Current));
  bridge1.virtualWrite(V47, String(bms.Battery_Power));
  bridge1.virtualWrite(V48, String(bms.Percent_Remain));
  Blynk.virtualWrite(V66, "update", 0, "Time", String(formattedTime) + " " + "WIB");
  Blynk.virtualWrite(V66, "update", 1, "battery_voltage", String(bms.Battery_Voltage) + " " + "V");
  Blynk.virtualWrite(V66, "update", 2, "battery_power", String(bms.Battery_Power) + " " + "W");
  Blynk.virtualWrite(V66, "update", 3, "Charge_Current", String(bms.Charge_Current) + " " + "A");
  Blynk.virtualWrite(V66, "update", 4, "Percent_Remain", String(bms.Percent_Remain) + " " + "%");
  Blynk.virtualWrite(V66, "update", 5, "capacity_remain", String(bms.Capacity_Remain) + " " + "Ah");
  Blynk.virtualWrite(V66, "update", 6, "nominal_capacity", String(bms.Nominal_Capacity) + " " + "Ah");
  Blynk.virtualWrite(V66, "update", 7, "cycle_count", String(bms.Cycle_Count) + " " + "x");
  Blynk.virtualWrite(V66, "update", 8, "cycle_capacity", String(bms.Cycle_Capacity) + " " + "Ah");
  Blynk.virtualWrite(V66, "update", 9, "battery_t1", String(bms.Battery_T1) + " " + "℃");
  Blynk.virtualWrite(V66, "update", 10, "battery_t2", String(bms.Battery_T2) + " " + "℃");
  Blynk.virtualWrite(V66, "update", 11, "mos_temp", String(bms.MOS_Temp) + " " + "℃");
  Blynk.virtualWrite(V66, "update", 12, "average_cell_voltage", String(bms.Average_Cell_Voltage) + " " + "V");
  Blynk.virtualWrite(V66, "update", 13, "delta_cell_voltage", String(bms.Delta_Cell_Voltage) + " " + "V");
  Blynk.virtualWrite(V66, "update", 14, "Balance_Curr", String(bms.Balance_Curr) + " " + "A");
  Serial.println(F("\nSELESAI"));
  }
  
}

