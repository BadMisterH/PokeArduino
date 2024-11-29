#include <Wire.h>
#include <Adafruit_PN532.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// Prototypes des fonctions
String receiveBluetoothJSON();
bool validateJSON(const String& jsonData);
bool waitForNFCCard();
void writeDataToNFC(const String& jsonData);
void readDataFromNFC(int numBlocks);
bool authenticateBlock(int block);
bool isCardPresent();

// Définition des broches et variables
#define PN532_IRQ (2)           // Broche IRQ
#define PN532_RESET (3)         // Broche Reset

// Bluetooth et communication
SoftwareSerial mySerial(10, 11); // RX, TX
const int STATE_PIN = 8;         // Broche "STATE" du HC-05
bool isConnected = false;        // Flag de connexion Bluetooth

// Configuration du tag NFC
unsigned long TagCards = 180661362;

// Instance PN532
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

// UID de la carte
uint8_t cardUID[7];
uint8_t cardUIDLength;

void setup() {
  // Initialisation des communications
  Serial.begin(115200);        // Moniteur série pour débogage
  mySerial.begin(9600);        // Communication Bluetooth

  // Configuration des broches
  pinMode(STATE_PIN, INPUT);

  // Initialisation du module NFC
  Serial.println("Initialisation du module PN532...");
  nfc.begin();
  
  if (!nfc.getFirmwareVersion()) {
    Serial.println("Erreur : Module PN532 non détecté !");
    while (1); // Arrêt du programme
  }
  
  nfc.SAMConfig(); // Configuration pour lecture RFID
  Serial.println("Système prêt. En attente de connexion Bluetooth...");
}

void loop() {
  // Vérification de la connexion Bluetooth
  if (digitalRead(STATE_PIN) == HIGH) {
    if (!isConnected) {
      Serial.println("Connexion Bluetooth établie !");
      isConnected = true;
    }

    // Réception des données Bluetooth
    String jsonData = receiveBluetoothJSON();
    
    if (jsonData.length() > 0) {
      Serial.println("Données JSON reçues : " + jsonData);
      
      // Attente et vérification de la carte NFC
      if (waitForNFCCard()) {
        // Écriture des données sur la carte NFC
        writeDataToNFC(jsonData);
      }
    }
  } else {
    // Gestion de la perte de connexion
    if (isConnected) {
      Serial.println("Perte de connexion Bluetooth...");
      isConnected = false;
    }
    delay(1000);
  }
}

// Fonction de réception des données JSON via Bluetooth
String receiveBluetoothJSON() {
  String jsonBuffer = "";
  unsigned long startTime = millis();
  
  // Attente des données avec timeout
  while (millis() - startTime < 2000) {
    if (mySerial.available()) {
      char incomingChar = mySerial.read();

      // Construction du JSON
      if (incomingChar == '{') {
        jsonBuffer = "{";  // Réinitialisation du buffer
      } else if (incomingChar == '}' && jsonBuffer.startsWith("{")) {
        jsonBuffer += incomingChar;
        break;  // Fin de la réception JSON
      } else if (jsonBuffer.startsWith("{")) {
        jsonBuffer += incomingChar;
      }
    }
    delay(10);  // Petit délai pour stabilité
  }

  // Nettoyage et validation du JSON
  jsonBuffer.trim();
  return validateJSON(jsonBuffer) ? jsonBuffer : "";
}

// Validation du format JSON
bool validateJSON(const String& jsonData) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonData);
  
  if (error) {
    Serial.print("Recherche de donnée JSON ");
    // Serial.println(error.f_str());
    return false;
  }
  
  return true;
}

// Vérifie si une carte NFC est présente
bool isCardPresent() {
  uint8_t uid[7];
  uint8_t uidLength;
  return nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
}

// Attente de la présence d'une carte NFC
bool waitForNFCCard() {
  Serial.println("Approchez votre carte NFC...");
  
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {  // Timeout de 10 secondes
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, cardUID, &cardUIDLength)) {
      // Calcul de l'ID de la carte
      unsigned long cardid = 0;
      for (int i = 0; i < min(4, cardUIDLength); i++) {
        cardid <<= 8;
        cardid |= cardUID[i];
      }

      Serial.print("Carte détectée. UID : ");
      Serial.println(cardid);

      // Vérification du tag autorisé
      if (cardid == TagCards) {
        Serial.println("Accès autorisé !");
        return true;
      } else {
        Serial.println("Carte non autorisée !");
      }
    }
    
    delay(500);  // Délai entre les tentatives
  }
  
  Serial.println("Aucune carte détectée dans le délai imparti.");
  return false;
}

// Authentifie le bloc spécifié
bool authenticateBlock(int block) {
  uint8_t key[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // Clé par défaut
  
  if (!nfc.mifareclassic_AuthenticateBlock(cardUID, cardUIDLength, block, 0, key)) {
    Serial.println("❌ Échec d'authentification pour le bloc " + String(block));
    return false;
  }
  return true;
}

// Écriture des données sur la carte NFC
void writeDataToNFC(const String& jsonData) {
  int totalLength = jsonData.length();
  int numBlocks = (totalLength + 15) / 16;
  int startingBlock = 4;

  for (int block = 0; block < numBlocks; block++) {
    uint8_t data[16] = {0};
      for (int i = 0; i < 16 && (block * 16 + i) < totalLength; i++) {
      data[i] = jsonData[block * 16 + i];
    }

    // Authentification du bloc
    if (!authenticateBlock(startingBlock + block)) {
      Serial.println("❌ Impossible d'authentifier le bloc " + String(startingBlock + block));
      return;
    }

    // Écriture des données dans le bloc NFC
    uint8_t success = nfc.mifareclassic_WriteDataBlock(startingBlock + block, data);
    if (success) {
      Serial.println("✅ Données écrites dans le bloc " + String(startingBlock + block));
    } else {
      Serial.println("❌ Échec de l'écriture dans le bloc " + String(startingBlock + block));
    }
  }

  // Vérification des données écrites après chaque bloc
  readDataFromNFC(numBlocks);
}

// Lecture des données de la carte NFC
void readDataFromNFC(int numBlocks) {
  for (int block = 4; block < 4 + numBlocks; block++) {
    uint8_t data[16];

    // Authentification du bloc
    if (!authenticateBlock(block)) {
      Serial.println("❌ Échec d'authentification pour le bloc " + String(block));
      return;
    }

    // Lecture des données du bloc
    if (nfc.mifareclassic_ReadDataBlock(block, data)) {
      Serial.print("Données lues du bloc " + String(block) + ": ");
      String readData = "";
      for (int i = 0; i < 16; i++) {
        if (data[i] != 0) {
          readData += (char)data[i];
        }
      }
      Serial.println(readData);
    } else {
      Serial.println("❌ Échec de lecture du bloc " + String(block));
    }
  }
}
