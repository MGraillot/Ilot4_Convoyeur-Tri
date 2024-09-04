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

// Valeurs calibrées à remplacer par les valeurs obtenues lors de la calibration
const int redMax = 13;    // Valeur maximale pour le rouge avec surface blanche
const int blueMax = 15;   // Valeur maximale pour le bleu avec surface blanche
const int redMin = 104;   // Valeur minimale pour le rouge avec surface noire
const int blueMin = 129;  // Valeur minimale pour le bleu avec surface noire

// Variables pour stocker les fréquences RGB
int redFrequency = 0;
int blueFrequency = 0;

// Prototypes des fonctions
int normalize(int value, int min, int max);
String detectColor(int red, int blue);

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
  myServo1.attach(12); // Attacher le servo1 à la broche GPIO 12 de l'ESP32
  myServo2.attach(14); // Attacher le servo2 à la broche GPIO 14 de l'ESP32

  // Initialiser les servos SG90 à 0 degré
  myServo1.write(0);
  myServo2.write(0);

  delay(1000); // Attendre 1 seconde pour l'initialisation
}

void loop() {
  // Lire la fréquence pour le rouge
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(100); // Délai pour la stabilisation de la lecture
  redFrequency = pulseIn(sensorOut, LOW);

  // Lire la fréquence pour le bleu
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  delay(100); // Délai pour la stabilisation de la lecture
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
    myServo1.write(90); // Déplacer le servo1 à 90°
    delay(5000);        // Attendre 5 secondes
    myServo1.write(0);  // Retourner le servo1 à 0°
  }

  // Si la couleur détectée est bleue, activer le servo2
  if (color == "Bleu") {
    myServo2.write(90); // Déplacer le servo2 à 90°
    delay(6000);        // Attendre 6 secondes
    myServo2.write(0);  // Retourner le servo2 à 0°
  }

  // Pause pour stabiliser la lecture
  delay(1000);
}

// Fonction pour normaliser les lectures
int normalize(int value, int min, int max) {
  if (max - min == 0)
    return 0;                           // Prévenir la division par zéro
  return map(value, min, max, 0, 255); // Convertir en échelle de 0 à 255
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