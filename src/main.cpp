#include <Arduino.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>

// Création des objets Servo
Servo myServo1;
Servo myServo2;

// Définir les broches du capteur TCS3200 pour l'ESP32
#define S0 4
#define S1 5
#define S2 18
#define S3 19
#define sensorOut 21

// Définition des broches pour le pilote du moteur pas à pas
#define DIR_PIN 33
#define STEP_PIN 32 

// Création d'une instance de AccelStepper pour contrôler le moteur pas à pas
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Valeurs calibrées à remplacer par les valeurs obtenues lors de la calibration
const int redMax = 13;
const int blueMax = 15;
const int redMin = 104;
const int blueMin = 129;

// Variables pour stocker les fréquences RGB
int redFrequency = 0;
int blueFrequency = 0;

// Prototypes des fonctions
int normalize(int value, int min, int max);
String detectColor(int red, int blue);

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
  myServo1.attach(12);  // Attacher le servo1 à la broche GPIO 12 de l'ESP32
  myServo2.attach(14);  // Attacher le servo2 à la broche GPIO 14 de l'ESP32

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
  // Lire la fréquence pour le rouge
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(100);  // Délai pour la stabilisation de la lecture
  redFrequency = pulseIn(sensorOut, LOW);

  // Lire la fréquence pour le bleu
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  delay(100);  // Délai pour la stabilisation de la lecture
  blueFrequency = pulseIn(sensorOut, LOW);

  // Normaliser les fréquences
  int normalizedRed = normalize(redFrequency, redMin, redMax);
  int normalizedBlue = normalize(blueFrequency, blueMin, blueMax);

  // Déterminer la couleur dominante
  String color = detectColor(normalizedRed, normalizedBlue);
  Serial.println("Couleur detectee: " + color);

  // Afficher les valeurs RGB normalisées
  Serial.print("Rouge: ");
  Serial.print(normalizedRed);
  Serial.print(" Bleu: ");
  Serial.println(normalizedBlue);

  // Si la couleur détectée est rouge, activer le servo1
  if (color == "Rouge") {
    myServo1.write(90);  // Déplacer le servo1 à 90°
    delay(5000);         // Attendre 5 secondes
    myServo1.write(0);   // Retourner le servo1 à 0°
  }

  // Si la couleur détectée est bleue, activer le servo2
  if (color == "Bleu") {
    myServo2.write(90);  // Déplacer le servo2 à 90°
    delay(6000);         // Attendre 6 secondes
    myServo2.write(0);   // Retourner le servo2 à 0°
  }
}

// Fonction pour normaliser les lectures
int normalize(int value, int min, int max) {
  if (max - min == 0)
    return 0;  // Prévenir la division par zéro
  return map(value, min, max, 0, 255);  // Convertir en échelle de 0 à 255
}

// Fonction pour détecter la couleur en fonction des valeurs RGB normalisées
String detectColor(int red, int blue) {
  if (red < 0 && blue < 0) {
    return "Couleur Indeterminee";
  } else if (red > blue) {
    return "Rouge";
  } else if (blue > red) {
    return "Bleu";
  }
  return "Couleur Indeterminee";
}