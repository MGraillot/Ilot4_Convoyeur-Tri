#include <WiFi.h>
#include <esp_now.h>

uint8_t macAddressPeer[] = {0x64, 0xB7, 0x08, 0x29, 0x13, 0x20};  // Adresse MAC de l'autre ESP32 (à changer pour chaque ESP)

void onDataReceive(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  Serial.print("Données reçues de : ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" | Longueur: ");
  Serial.println(len);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Statut de l'envoi : ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Succès" : "Échec");
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // Ajout de message pour vérifier l'état de déconnexion du Wi-Fi
  if (WiFi.status() == WL_DISCONNECTED) {
    Serial.println("Wi-Fi déconnecté avec succès !");
  } else {
    Serial.println("Erreur lors de la déconnexion du Wi-Fi.");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erreur lors de l'initialisation d'ESP-NOW");
    return;
  } else {
    Serial.println("ESP-NOW initialisé avec succès !");
  }

  esp_now_register_recv_cb(onDataReceive);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, macAddressPeer, 6);
  peerInfo.channel = 1; // Essayez avec différents canaux si nécessaire
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erreur lors de l'ajout du peer");
    return;
  } else {
    Serial.println("Peer ajouté avec succès !");
  }
}

void loop() {
  uint8_t data = 42; // Donnée simple pour tester
  esp_err_t result = esp_now_send(macAddressPeer, &data, sizeof(data));

  if (result == ESP_OK) {
    Serial.println("Message envoyé !");
  } else {
    Serial.println("Échec de l'envoi du message");
  }

  delay(2000);
}