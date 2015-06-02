/**
 * Implementierung: Kugelfall
 *
 * @author Chris Köcher
 * @author Philipp Schlag
 */

#include <Servo.h> 
Servo myservo;

const int maxNumPhotoValues = 7;

enum ControllerState {
  IDLE,
  WAIT_FOR_MEMORY,
  WAIT_FOR_APPROX,
  CALC_WAIT_DROP
};

enum MemoryState {
  INIT,
  NOT_READY,
  READY
};

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
  MemoryState state;
  int photoLastIndex;
  int photoCount; // 0 initial; 1-6: x-ter Photowert nach letztem Hallwert
  timedValue photoValues[maxNumPhotoValues]; //TODO; wie viele?
  timedValue hallValue; //TODO: wie viele?
};

/**
 * Diese Datenstruktur wird verwendet, um den approximiert
 * nächstmöglichen Zeitpunkt zum Abwurf der Kugel zu speichern.
 **/
struct Approximator {
  bool isValid;
  timedValue velocity[2]; //TODO: hier oder in Approximator???
  timedValue acceleration;
  timedValue angle;
  int nextDropTime;
};

//zu verwendende Pins
const int pin_in_trigger = 2;
const int pin_in_photo = 4;
const int pin_in_hall = 5;
const int pin_out_servo = 9;
const int pin_out_led1 = 12;
const int pin_out_led2 = 13;

int val_trigger = 0;
int val_photo = 0;
int val_hall = 0;
int val_millis = 0;
int pos = 0;
int time_start = 0;
int time_end = 0;
int time_diff;
Memory defaultMemory;
Approximator defaultApprox;
ControllerState defaultControllerState;

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
  
  //initialisiere Controller
  defaultControllerState = IDLE;
  
  //initialisiere den Wertespeicher und Approximator
  initMemory();
  initApproximator();
  
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
//  val = digitalRead(pin_in_trigger);
//  if(val == HIGH) {
//    drop();
//  }
  readInputs();
  updateMemory();
  switch(defaultControllerState) {
    case IDLE:
      if(val_trigger == HIGH) {
        defaultControllerState = WAIT_FOR_MEMORY;
      } else {
        break;
      }
    case WAIT_FOR_MEMORY:
      if(defaultMemory.state == READY) {
        defaultControllerState = WAIT_FOR_APPROX;
      } else {
        break;
      }
    case WAIT_FOR_APPROX:
      // TODO APPROXIMIEREN-Aufruf, update memory?
      if(defaultApprox.isValid) {
        defaultControllerState = CALC_WAIT_DROP;
      } else {
        break;
      }
    case CALC_WAIT_DROP:
      // TODO calc drop time, update memory?
      // TODO wait for drop (and update MEMORY) -> WAIT_FOR_APPROX?
      // TODO drop -> defaultControllerState = IDLE;
      break;
  }
}

/**
 * Werte einlesen.
 **/
void readInputs() {
  val_millis = millis();
  val_trigger = digitalRead(pin_in_trigger);
  val_photo = digitalRead(pin_in_photo);
  val_hall = digitalRead(pin_in_hall);
}

/**
 * Initialisiere den Approximator.
 **/
void initApproximator() {
  defaultApprox.isValid = false;
  for(int i=0; i<2; i++) {
    defaultApprox.velocity[i].time = -1;
  }
  defaultApprox.acceleration.time = -1;
  defaultApprox.angle.time = -1;  
  defaultApprox.nextDropTime = -1;
}

/**
 * Initialisiere den Wertespeicher.
 **/
void initMemory() {
  defaultMemory.state = INIT;
  defaultMemory.photoLastIndex = maxNumPhotoValues-1;
  defaultMemory.photoCount = 0;
  digitalWrite(pin_out_led1, LOW);
  for(int i=0; i<7; i++) {
    defaultMemory.photoValues[i].time = -1;
    //defaultMemory.photoValues[i].value = -1.0;
  }
  defaultMemory.hallValue.time = millis();
  defaultMemory.hallValue.value = digitalRead(pin_in_hall);
  
  //TODO
  //defaultMemory.isReady = true;
  //digitalWrite(pin_out_led1, HIGH);
}

/**
 * Aktualisiere den Wertespeicher.
 **/
void updateMemory() {
  switch(defaultMemory.state) {
    case INIT:
      if(val_photo != (int)defaultMemory.photoValues[defaultMemory.photoLastIndex].value) {
        addPhotoValue();
      }
      if((defaultMemory.photoCount > 0) && (val_hall != (int)defaultMemory.hallValue.value)) {
        defaultMemory.hallValue.time = val_millis;
        defaultMemory.hallValue.value = val_hall;
        defaultMemory.photoCount = 0;
        defaultMemory.state = NOT_READY;
      }
      break;
    case NOT_READY:
      if(val_hall != (int)defaultMemory.hallValue.value) {
        defaultMemory.hallValue.time = val_millis; // TODO ZEITPUNKT???
        defaultMemory.hallValue.value = val_hall;
      }
      if(val_photo != (int)defaultMemory.photoValues[defaultMemory.photoLastIndex].value) {
        addPhotoValue();
        defaultMemory.state = READY;
        digitalWrite(pin_out_led1, HIGH);
      }
      break;
    case READY:
      if(val_photo != (int)defaultMemory.photoValues[defaultMemory.photoLastIndex].value) {
        addPhotoValue();
      }
      if(val_hall != (int)defaultMemory.hallValue.value) {
        defaultMemory.hallValue.time = val_millis;
        defaultMemory.hallValue.value = val_hall;
        defaultMemory.photoCount = 0;
        defaultMemory.state = NOT_READY;
        digitalWrite(pin_out_led1, LOW);
      }
  }
}

void addPhotoValue() {
  defaultMemory.photoLastIndex = (defaultMemory.photoLastIndex + 1) % maxNumPhotoValues;
  defaultMemory.photoValues[defaultMemory.photoLastIndex].time = val_millis;
  defaultMemory.photoValues[defaultMemory.photoLastIndex].value = val_photo;
  defaultMemory.photoCount++;
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

/**
 * Verzögerung inklusive Erfassung der Messwerte
 **/
void busyDelay(int time) {
  int waitUntil = millis() + time;
  while(millis() < waitUntil) {
    readInputs();
    // TODO Update Memory
  }
}
