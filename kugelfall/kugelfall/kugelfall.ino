/**
 * Implementierung: Kugelfall
 *
 * @author Chris Köcher
 * @author Philipp Schlag
 */

#include <Servo.h> 
Servo myservo;

const int maxNumPhotoValues = 13;         // maximale Anzahl der im Memory gespeicherten Photowerte
const float velocityThreshold = 400.0;    // Grenzgeschwindigkeit zum Umschalten der Approximationsmethode
const int timeDelaySlow = 391;            // Auslöse- und Fallzeit-Verzögerung (bei langsamer Drehung)
const int timeDelayFast = 391 + 100 + 8;  // Auslöse- und Fallzeit-Verzögerung (bei schneller Drehung)

/**
 * Die zentralen Zustände des Steueralgorithmus.
 **/
enum ControllerState {
  IDLE,
  WAIT_APPROX,
  CALC_WAIT_DROP
};

/**
 * Die Zustände des Wertespeichers/Memorys
 **/
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
  unsigned long time;
  int value;
};

/**
 * Diese Datenstruktur wird verwendet, um die aktuellsten
 * ausgelesenen Werte der Sensoren sowie einige daraus berechnete
 * Zahlen (Geschwindigkeit, Beschleunigung und Winkel) zu speichern.
 **/
struct Memory {
  MemoryState state;
  int photoLastIndex;   // 0..maxNumPhotoValues-1: Index des aktuellsten Photowertes
  int photoCount;       // 0 initial; 1..6: x-ter Photowert nach letztem Hallwert gelesen
  int photoTotalCount;  // 0..INTMAX: Anzahl gemessener Photowerte seit Initialisierung
  timedValue photoValues[maxNumPhotoValues];
  timedValue hallValue;
};

/**
 * Diese Datenstruktur wird verwendet, um den approximiert
 * nächstmöglichen Zeitpunkt zum Abwurf der Kugel zu speichern.
 **/
struct Approximator {
  bool isValid;
  float velocity;
  float acceleration;
  int angle;
  unsigned long time;
  unsigned long nextDropTime;
};

//zu verwendende Pins
const int pin_in_trigger = 2;
const int pin_in_photo = 4;
const int pin_in_hall = 5;
const int pin_out_servo = 9;
const int pin_out_led1 = 12;
const int pin_out_led2 = 13;

// aktuelle Werte der Input-Pins und zugehörige Zeit (in Millisekunden)
int val_trigger = 0;
int val_photo = 0;
int val_hall = 0;
unsigned long val_millis = 0;

// Zentrale Datenstrukturen
Memory defaultMemory;
Approximator defaultApprox;
ControllerState defaultControllerState;

// Signalisiert die Notwendigkeit einer (erneuten) Approximation der Werte
boolean doApproximate = true;

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
  delay(100);
  myservo.write(15);
  
  Serial.begin(9600);
  delay(100);
}

/**
 * Initialisiere den Wertespeicher.
 **/
void initMemory() {
  defaultMemory.state = INIT;
  defaultMemory.photoLastIndex = maxNumPhotoValues-1;
  defaultMemory.photoCount = 0;
  defaultMemory.photoTotalCount = 0;
  digitalWrite(pin_out_led1, LOW);
  for(int i=0; i<maxNumPhotoValues; i++) {
    defaultMemory.photoValues[i].time = 0;        
    defaultMemory.photoValues[i].value = -1;
  }
  defaultMemory.hallValue.time = millis();
  defaultMemory.hallValue.value = digitalRead(pin_in_hall);
}

/**
 * Initialisiere den Approximator.
 **/
void initApproximator() {
  defaultApprox.isValid = false;
  defaultApprox.time = 0;
  defaultApprox.nextDropTime = 0;
  doApproximate = true;
}

/**
 * Programm zum Testen des Versuchsaufbaus
 **/
/*void loop() {
  readInputs();
  if(val_trigger == HIGH) {
    drop();
  }
}*/

/**
 * Die Hauptschleife
 **/
void loop() {
  // Einlesen der aktuellen Sensorwerte und Aktualisieren des Memorys
  readInputs();
  boolean newValue = updateMemory();
  switch(defaultControllerState) {
    case IDLE:
      // Leerlauf - Warten auf das Drücken des Triggers
      if(val_trigger == HIGH) {
        defaultControllerState = WAIT_APPROX;
        doApproximate = true;
      } else {
        break;
      }
    case WAIT_APPROX:
      // Approximieren von Position, Geschwindigkeit und Verzögerung der drehenden Scheibe
      if(defaultMemory.state == READY) {
        if(doApproximate || newValue) {
          approximate();
        }
        
        if(defaultApprox.isValid) {
          defaultControllerState = CALC_WAIT_DROP;
        } else {
          break;
        }
      } else {
        break;
      }      
    case CALC_WAIT_DROP:
      // frühest möglichen Abwurfzeitpunkt berechnen
      calcDropTime();
      // warten auf Abwurfzeitpunkt, ggf. Aktualisieren der Approximation und des Abwurfzeitpunktes
      if(busyWaitForDrop()) {
        drop();
        // Serial.print("Geschwindigkeit: ");
        Serial.println(defaultApprox.velocity);
        defaultControllerState = IDLE;
      } else {
        defaultControllerState = WAIT_APPROX;
        doApproximate = true;
      }
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
 * Aktualisiere den Wertespeicher.
 *
 * @return true, falls neuer Photowert vorliegt, false sonst
 **/
boolean updateMemory() {
  boolean newPhotoValue = false;
  switch(defaultMemory.state) {
    case INIT:
      if(val_photo != defaultMemory.photoValues[defaultMemory.photoLastIndex].value) {
        addPhotoValue();
        newPhotoValue = true;
      }
      if((defaultMemory.photoCount > 0) && (val_hall != defaultMemory.hallValue.value)) {
        defaultMemory.hallValue.time = val_millis;
        defaultMemory.hallValue.value = val_hall;
        defaultMemory.photoCount = 0;
        defaultMemory.state = NOT_READY;
      }
      break;
    case NOT_READY:
      if(val_hall != defaultMemory.hallValue.value) {
        defaultMemory.hallValue.time = val_millis;
        defaultMemory.hallValue.value = val_hall;
      }
      if(val_photo != defaultMemory.photoValues[defaultMemory.photoLastIndex].value) {
        addPhotoValue();
        defaultMemory.state = READY;
        digitalWrite(pin_out_led1, HIGH);
        newPhotoValue = true;
      }
      break;
    case READY:
      if(val_photo != defaultMemory.photoValues[defaultMemory.photoLastIndex].value) {
        addPhotoValue();
        newPhotoValue = true;
      }
      if(val_hall != defaultMemory.hallValue.value) {
        defaultMemory.hallValue.time = val_millis;
        defaultMemory.hallValue.value = val_hall;
        defaultMemory.photoCount = 0;
        defaultMemory.state = NOT_READY;
        digitalWrite(pin_out_led1, LOW);
      }
      break;
  }
  return newPhotoValue;
}

/**
 * Füge neuen Photowert dem Memory hinzu.
 **/
void addPhotoValue() {
  defaultMemory.photoLastIndex = (defaultMemory.photoLastIndex + 1) % maxNumPhotoValues;
  defaultMemory.photoValues[defaultMemory.photoLastIndex].time = val_millis;
  defaultMemory.photoValues[defaultMemory.photoLastIndex].value = val_photo;
  defaultMemory.photoCount++;
  defaultMemory.photoTotalCount++;
}

/**
 * Scheibenposition, Geschwindigkeit und Verzögerung approximieren
 **/
void approximate() {
  // überprüfe, ob genügend aktuelle Photowerte vorliegen
  if(defaultMemory.photoTotalCount < maxNumPhotoValues) {
    defaultApprox.isValid = false;
    digitalWrite(pin_out_led2, LOW);
    return;
  }
  
  // überprüfe, ob die Beschleunigung/Verzögerung in Ordnung ist
  if(!validateSpeedUp()) {
    // der tatsächliche quot-Wert des aktuellen Abschnittes kann nur größer sein
    // plötzlicher Stopp innerhalb eines 30°-Segmentes wird erkannt
    defaultApprox.isValid = false;
    digitalWrite(pin_out_led2, LOW);
    return;
  }
  float t1, t2, quot;
  t1 = (float)defaultMemory.photoValues[defaultMemory.photoLastIndex].time - (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-1)%maxNumPhotoValues].time;
  for(int i = 1; i < maxNumPhotoValues-1; i++) {
    t2 = t1;
    t1 = (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-i)%maxNumPhotoValues].time - (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-i-1)%maxNumPhotoValues].time;
    quot = t2/t1;
    if(quot < 0.5 || quot > 1.5) {
      defaultApprox.isValid = false;
      digitalWrite(pin_out_led2, LOW);
      return;
    }
  }
  
  // Berechnung des Winkels
  defaultApprox.angle = 15 + defaultMemory.hallValue.value*180 + (defaultMemory.photoCount-1)*30;
  
  // Geschwindigkeit
  defaultApprox.velocity = ((float)defaultMemory.photoValues[defaultMemory.photoLastIndex].time - (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-12)%maxNumPhotoValues].time) / 12.0;
  
  // Geschwindigkeit im gültigen Bereich?
  if(defaultApprox.velocity < 20.83 || defaultApprox.velocity > 833.33) {
    defaultApprox.isValid = false;
    digitalWrite(pin_out_led2, LOW);
    return;
  }
  
  // Verzögerung
  if(defaultApprox.velocity < velocityThreshold) {
    // Approximation bei schneller Drehung
    defaultApprox.acceleration = ((float)defaultMemory.photoValues[defaultMemory.photoLastIndex].time - (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-1)%maxNumPhotoValues].time - (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-11)%maxNumPhotoValues].time + (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-12)%maxNumPhotoValues].time) / 11.0;
  } else {
    // Approximation bei langsamer Drehung
    defaultApprox.acceleration = ((float)defaultMemory.photoValues[defaultMemory.photoLastIndex].time + (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-2)%maxNumPhotoValues].time - 2*(float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-1)%maxNumPhotoValues].time);
  }
  if(defaultApprox.acceleration < 0) {
    defaultApprox.acceleration = 0;
  }
  
  // fertig
  defaultApprox.time = defaultMemory.photoValues[defaultMemory.photoLastIndex].time;
  defaultApprox.isValid = true;
  digitalWrite(pin_out_led2, HIGH);
  doApproximate = false;
}

/**
 * Überprüft, ob die Bewegung im aktuellen 30°-Segment noch gültig ist oder schon zu lange dauert
 *
 * @return false, falls keine oder zu geringe Bewegung im aktuellen Segment gemessen, sonst true
 **/
boolean validateSpeedUp() {
  float t1, t2, quot;
  t2 = (float)millis() - (float)defaultMemory.photoValues[defaultMemory.photoLastIndex].time;
  t1 = (float)defaultMemory.photoValues[defaultMemory.photoLastIndex].time - (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-1)%maxNumPhotoValues].time;
  quot = t2/t1;
  return quot <= 1.5;
}

/**
 * Berechne die nächstmögliche Fallzeit
 **/
void calcDropTime() {
  int timeDelay;
  if(defaultApprox.velocity < velocityThreshold) {
    timeDelay = timeDelayFast;
  } else {
    timeDelay = timeDelaySlow;
  }
  unsigned long n = (12 - (defaultApprox.angle + 15) / 30) % 12;
  unsigned long tmin, current = millis();
  do {
    tmin = defaultApprox.time + n * defaultApprox.velocity + defaultApprox.velocity / 2 + (n * (n+1))/2 * defaultApprox.acceleration;
    n += 12;
  } while(tmin < current + timeDelay);
  defaultApprox.nextDropTime = tmin - timeDelay;
}

/**
 * Warte auf den Abwurfzeitpunkt.
 * Lese währenddessen die Sensorwerte ein und akualisiere ggf. den Abwurfzeitpunkt.
 *
 * @return true, wenn Abwurfzeitpunkt erfolgreich erreicht, false, wenn abgebrochen werden soll
 **/
boolean busyWaitForDrop() {
  unsigned long current;
  while(true) {
    current = millis();
    if(current >= defaultApprox.nextDropTime) {
      // Abwurfzeitpunkt erreicht
      return true;
    }
    readInputs();
    if(updateMemory()) {
      // neue Approximation aufgrund neuer Sensorwerte
      approximate();
      if(!defaultApprox.isValid) {
        return false;
      }
      calcDropTime();
    } else {
      if(!validateSpeedUp()) {
        // der tatsächliche quot-Wert des aktuellen Abschnittes kann nur größer sein
        // plötzlicher Stopp innerhalb eines 30°-Segmentes wird erkannt
        defaultApprox.isValid = false;
        digitalWrite(pin_out_led2, LOW);
        return false;
      }
    }
  }
}

/**
 * Diese Prozedur wird verwendet, um das Fallrohr zu öffnen und
 * anschließend für die nächste Kugel vorzubereiten.
 **/
void drop() {
  //eigentliche Prozedur
  myservo.write(15);
  for(int pos = 15; pos <= 20; pos += 1) {
    myservo.write(pos);
    busyDelay(20);
  }
  
  //zurücksetzen
  busyDelay(100);
  myservo.write(0);
  busyDelay(100);
  myservo.write(15);
}

/**
 * Verzögerung inklusive Erfassung der Messwerte
 *
 * @param time Dauer der Verzögerung in Millisekunden
 **/
void busyDelay(unsigned long time) {
  unsigned long waitUntil = millis() + time;
  while(millis() < waitUntil) {
    readInputs();
    updateMemory();
  }
}

/**
 * Debugging-Methode
 **/
void printPhotoValues() {
  Serial.print("time: ");
  Serial.println(defaultMemory.photoValues[defaultMemory.photoLastIndex].time);
  Serial.print("photoCount: ");
  Serial.println(defaultMemory.photoCount);
  Serial.println("photoValue - time");
  for(int i = 0; i < maxNumPhotoValues; i++) {
    if((defaultMemory.photoLastIndex+1+i)%maxNumPhotoValues < defaultMemory.photoTotalCount) {
      Serial.print(defaultMemory.photoValues[(defaultMemory.photoLastIndex+1+i)%maxNumPhotoValues].value);
      Serial.print(" - ");
      Serial.println(defaultMemory.photoValues[(defaultMemory.photoLastIndex+1+i)%maxNumPhotoValues].time);
    }
  }
}
