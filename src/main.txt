#include <Arduino.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>

// Création des objets Servo
Servo myServo1;
Servo myServo2;

// Définir les broches du capteur TCS3200 pour l'ESP32
#define S0 18
#define S1 5
#define S2 17
#define S3 16
#define sensorOut 26

// Définition des broches pour le pilote du moteur pas à pas
#define DIR_PIN 15
#define STEP_PIN 2 

// Création d'une instance de AccelStepper pour contrôler le moteur pas à pas
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Valeurs calibrées à remplacer par les valeurs obtenues lors de la calibration
const int redMax = 26;
const int blueMax = 21;
const int redMin = 182;
const int blueMin = 148;

// Variables pour stocker les fréquences RGB
int redFrequency = 0;
int blueFrequency = 0;

// Prototypes des fonctions
int normalize(int value, int min, int max);
String detectColor(int red, int blue);
String checkColorConsistency(int retries);

// Tâche FreeRTOS pour la gestion continue du moteur et ne plus avoir de problèmes avec les delays
void TaskStepper(void *pvParameters) {
  while (1) {
    stepper.runSpeed();  // Faire tourner le moteur en continu
    vTaskDelay(1);       // Petite pause pour ne pas monopoliser le CPU
  }
}

void setup() {
  // Initialiser la communication série
  Serial.begin(115200);

  // Configurer les broches du capteur comme sorties
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Configurer la pleine échelle de sortie (100%)
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Attacher les servos aux broches GPIO
  myServo1.attach(22);  // Attacher le servo1 à la broche GPIO 12 de l'ESP32
  myServo2.attach(23);  // Attacher le servo2 à la broche GPIO 14 de l'ESP32

  // Initialiser les servos SG90 à 0 degré
  myServo1.write(0);
  myServo2.write(0);

  delay(1000);  // Attendre 1 seconde pour l'initialisation

  // Initialiser le moteur pas à pas
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  stepper.setMaxSpeed(1000);  // Définir la vitesse maximale du moteur
  stepper.setSpeed(200);      // Démarrer le moteur à une vitesse constante

  // Création d'une tâche FreeRTOS pour la gestion continue du moteur
  xTaskCreatePinnedToCore(
      TaskStepper,      // Fonction de la tâche
      "TaskStepper",    // Nom de la tâche
      10000,            // Taille de la pile de la tâche
      NULL,             // Paramètres d'entrée de la tâche
      1,                // Priorité de la tâche
      NULL,             // Gestion de la tâche (non utilisé ici)
      0);               // Épingle la tâche sur le cœur 0 de l'ESP32
}

void loop() {
  // Vérifier la couleur avec plusieurs prises pour garantir la cohérence
  String color = checkColorConsistency(3);
  Serial.println("Couleur detectee: " + color);

  // Si la couleur détectée est rouge, activer le servo1
  if (color == "Rouge") {
    myServo1.write(90);  // Déplacer le servo1 à 90°
    delay(5000);         // Attendre 5 secondes
    myServo1.write(0);   // Retourner le servo1 à 0°
  }

  // Si la couleur détectée est bleue, activer le servo2
  if (color == "Bleu") {
    myServo2.write(90);  // Déplacer le servo2 à 90°
    delay(7000);         // Attendre 6 secondes
    myServo2.write(0);   // Retourner le servo2 à 0°
  }
}

// Fonction pour lire les fréquences du capteur et normaliser les valeurs
void readSensor() {
  // Lire la fréquence pour le rouge
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(50);  // Délai pour la stabilisation de la lecture
  redFrequency = pulseIn(sensorOut, LOW);

  // Lire la fréquence pour le bleu
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  delay(50);  // Délai pour la stabilisation de la lecture
  blueFrequency = pulseIn(sensorOut, LOW);
}

// Fonction pour normaliser les lectures
int normalize(int value, int min, int max) {
  if (max - min == 0)
    return 0;  // Prévenir la division par zéro
  return map(value, min, max, 0, 255);  // Convertir en échelle de 0 à 255
}

// Fonction pour détecter la couleur en fonction des valeurs RGB normalisées
String detectColor(int red, int blue) {
  if (red <= 50 && blue <= 50) {
    return "Couleur Indeterminee";
  } else if (red > blue) {
    return "Rouge";
  } else if (blue > red) {
    return "Bleu";
  }
  return "Couleur Indeterminee";
}

// Fonction pour vérifier la cohérence des prises de couleur
String checkColorConsistency(int retries) {
  String finalColor = "Couleur Indeterminee";
  int redSum = 0;
  int blueSum = 0;

  for (int i = 0; i < retries; i++) {
    readSensor();

    // Normaliser les fréquences
    int normalizedRed = normalize(redFrequency, redMin, redMax);
    int normalizedBlue = normalize(blueFrequency, blueMin, blueMax);

    // Ajouter les valeurs pour une moyenne
    redSum += normalizedRed;
    blueSum += normalizedBlue;
  }

  // Calculer la moyenne des valeurs
  int avgRed = redSum / retries;
  int avgBlue = blueSum / retries;

  // Détecter la couleur dominante
  finalColor = detectColor(avgRed, avgBlue);
  return finalColor;
}