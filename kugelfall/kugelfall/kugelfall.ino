/**
 * Implementierung: Kugelfall
 *
 * @author Chris Köcher
 * @author Philipp Schlag
 */

#include <Servo.h> 
Servo myservo;

/**
 * Eine Datenstruktur, die einem konkreten Zeitpunkt einen
 * Wert zu ordnet.
 **/
struct timedValue {
  int time;
  float value;
};

/**
 * Diese Datenstruktur wird verwendet, um die aktuellsten
 * ausgelesenen Werte der Sensoren sowie einige daraus berechnete
 * Zahlen (Geschwindigkeit, Beschleunigung und Winkel) zu speichern.
 **/
struct Memory {
  bool isReady;
  int photoCount;
  timedValue photoValues[12]; //TODO; wie viele?
  timedValue hallValue; //TODO: wie viele?
  timedValue velocity[2]; //TODO: hier oder in Approximator???
  timedValue acceleration;
  timedValue angle;
};

/**
 * Diese Datenstruktur wird verwendet, um den approximiert
 * nächstmöglichen Zeitpunkt zum Abwurf der Kugel zu speichern.
 **/
struct Approximator {
  bool isReady;
  int nextDropTime;
};

//zu verwendende Pins
const int pin_in_trigger = 2;
const int pin_in_photo = 4;
const int pin_in_hall = 5;
const int pin_out_servo = 9;
const int pin_out_led1 = 12;
const int pin_out_led2 = 13;

int val = 0;
int pos = 0;
int time_start = 0;
int time_end = 0;
int time_diff;
Memory defaultMemory;
Approximator defaultApprox;

/**
 * Führe die Initialisierung aus.
 **/
void setup() {
  //initialisiere die Pins
  pinMode(pin_in_trigger, INPUT);
  pinMode(pin_in_photo, INPUT);
  pinMode(pin_in_hall, INPUT);
  pinMode(pin_out_led1, OUTPUT);
  pinMode(pin_out_led2, OUTPUT);
  
  //initialisiere die LEDs
  digitalWrite(pin_out_led1, LOW);
  digitalWrite(pin_out_led2, LOW);
  
  //initialisiere den Wertespeicher
  
  //initialisiere den Motor
  myservo.attach(pin_out_servo);
  myservo.write(0);
  Serial.begin(9600);
  delay(1000);
}

/**
 * Die Hauptschleife
 **/
void loop() {
  val = digitalRead(pin_in_trigger);
  if(val == HIGH) {
    drop();
  }
}

/**
 * Initialisiere den Wertespeicher.
 **/
void initMemory() {
  defaultMemory.isReady = false;
  digitalWrite(pin_out_led1, LOW);
  
  //TODO
  
  defaultMemory.isReady = true;
  digitalWrite(pin_out_led1, HIGH);
}

/**
 * Diese Prozedur wird verwendet, um das Fallrohr zu öffnen und
 * anschließend für die nächste Kugel vorzubereiten.
 **/
void drop() {
  time_start = millis();//debugging
  
  //eigentliche Prozedur
  myservo.write(17);
  for(pos = 17; pos <= 20; pos += 1) {
    myservo.write(pos);
    delay(20);
  }
  
  //debugging
  time_end = millis();
  time_diff = time_end - time_start;
  Serial.println(time_diff);
  
  //zurücksetzen
  delay(50);
  myservo.write(0);
  delay(50);
  myservo.write(17);
}
