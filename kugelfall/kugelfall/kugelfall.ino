/**
 * Implementierung: Kugelfall
 *
 * @author Chris Köcher
 * @author Philipp Schlag
 */

#include <Servo.h> 
Servo myservo;

//const int maxNumPhotoValues = 7;
const int maxNumPhotoValues = 13;
const int timeDelaySlow = 391 + 80;
const int timeDelayFast = 391 + 115;
const int timeDelay = 391 + 90; //TODO!!!

enum ControllerState {
  IDLE,
  //WAIT_FOR_MEMORY,
  //WAIT_FOR_APPROX,
  WAIT_APPROX,
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
  int photoLastIndex;
  int photoCount; // 0 initial; 1-6: x-ter Photowert nach letztem Hallwert
  int photoTotalCount; // 0..INTMAX: Anzahl gemessener Photowerte seit Initialisierung
  timedValue photoValues[maxNumPhotoValues];
  timedValue hallValue; //TODO: wie viele?
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

int val_trigger = 0;
int val_photo = 0;
int val_hall = 0;
unsigned long val_millis = 0;
int pos = 0;
unsigned long time_start = 0;
unsigned long time_end = 0;
unsigned long time_diff;
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

/*void loop() {
  time_start = millis();
  readInputs();
  updateMemory();
  approximate();
  calcDropTime();
  time_end = millis();
  Serial.println(time_end - time_start);
}*/



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
        //defaultControllerState = WAIT_FOR_MEMORY;
        defaultControllerState = WAIT_APPROX;
      } else {
        break;
      }
//    case WAIT_FOR_MEMORY:
//      if(defaultMemory.state == READY) {
//        defaultControllerState = WAIT_FOR_APPROX;
//      } else {
//        break;
//      }
//    case WAIT_FOR_APPROX:
//      // TODO APPROXIMIEREN-Aufruf, update memory?
//      
//      if(defaultApprox.isValid) {
//        defaultControllerState = CALC_WAIT_DROP;
//      } else {
//        break;
//      }
    case WAIT_APPROX:
      if(defaultMemory.state == READY) {
        approximate();
        
        if(defaultApprox.isValid) {
          defaultControllerState = CALC_WAIT_DROP;
        } else {
          break;
        }
      } else {
        break;
      }      
    case CALC_WAIT_DROP:
      calcDropTime();
      
      if(busyWaitForDrop()) {
        drop();
        defaultControllerState = IDLE;
      } else {
        defaultControllerState = WAIT_APPROX;
      }
      break;
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
      Serial.print(defaultMemory.photoValues[(defaultMemory.photoLastIndex+1+i)%maxNumPhotoValues].time);
    }
  }
}

/**
 * Scheibenposition, Geschwindigkeit und Verzögerung approximieren
 **/
void approximate() {
  // überprüfe, ob genügend aktuelle Photowerte vorliegen
  if(defaultMemory.photoTotalCount < maxNumPhotoValues) {//Serial.println("bla1");
    defaultApprox.isValid = false;
    digitalWrite(pin_out_led2, LOW);
    return;
  }
  
  // überprüfe, ob die Beschleunigung in Ordnung ist
  float t1, t2, quot;
  t2 = millis() - defaultMemory.photoValues[defaultMemory.photoLastIndex].time;
  t1 = defaultMemory.photoValues[defaultMemory.photoLastIndex].time - defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-1)%maxNumPhotoValues].time;
  quot = t2/t1;
  if(quot > 1.5) {//Serial.print(millis());Serial.print(" ");Serial.print(defaultMemory.photoValues[defaultMemory.photoLastIndex].time);Serial.print(" ");Serial.println(defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-1)%maxNumPhotoValues].time);
    // der tatsächliche quot-Wert des aktuellen Abschnittes kann nur größer sein
    // plötzlicher Stopp innerhalb eines 30°-Segmentes wird erkannt
    defaultApprox.isValid = false;
    digitalWrite(pin_out_led2, LOW);
    return;
  }
  for(int i = 1; i < maxNumPhotoValues-1; i++) {
    t2 = t1;
    t1 = defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-i)%maxNumPhotoValues].time - defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-i-1)%maxNumPhotoValues].time;
    quot = t2/t1;
    if(quot < 0.5 || quot > 1.5) {Serial.print("bla3 ");Serial.println(quot);
      defaultApprox.isValid = false;
      digitalWrite(pin_out_led2, LOW);
      return;
    }
  }
  
  // Berechnung des Winkels
  defaultApprox.angle = 15 + defaultMemory.hallValue.value*180 + (defaultMemory.photoCount-1)*30;
  
  
  // Geschwindigkeit
  defaultApprox.velocity = ((float)defaultMemory.photoValues[defaultMemory.photoLastIndex].time - (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-6)%maxNumPhotoValues].time) / 6.0;
  //defaultApprox.velocity = ((float)defaultMemory.photoValues[defaultMemory.photoLastIndex].time - (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-12)%maxNumPhotoValues].time) / 12.0;
  
  // Geschwindigkeit im gültigen Bereich?
  if(defaultApprox.velocity < 20.83 || defaultApprox.velocity > 833.33) {//Serial.println("bla4 ");Serial.println(defaultApprox.velocity);
    defaultApprox.isValid = false;
    digitalWrite(pin_out_led2, LOW);
    return;
  }
  
  // Verzögerung
  //  defaultApprox.acceleration = ((float)defaultMemory.photoValues[defaultMemory.photoLastIndex].time + (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-6)%maxNumPhotoValues].time - 2*(float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-3)%maxNumPhotoValues].time) / 3.0;
  defaultApprox.acceleration = ((float)defaultMemory.photoValues[defaultMemory.photoLastIndex].time + (float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-12)%maxNumPhotoValues].time - 2*(float)defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-6)%maxNumPhotoValues].time) / 6.0;
  if(defaultApprox.acceleration < 0 || defaultApprox.acceleration > 12345) {Serial.print("bla5");Serial.println(defaultApprox.acceleration);
    defaultApprox.isValid = false;
    digitalWrite(pin_out_led2, LOW);
    Serial.print("bad acceleration: ");
    Serial.println(defaultApprox.acceleration);
    printPhotoValues();
    return;
  }
  
  
  // fertig
  defaultApprox.time = defaultMemory.photoValues[defaultMemory.photoLastIndex].time;
  defaultApprox.isValid = true;
  digitalWrite(pin_out_led2, HIGH);
}

/**
 * Berechne die nächstmögliche Fallzeit
 **/
void calcDropTime() {
  unsigned long n = (12 - (defaultApprox.angle + 15) / 30) % 12;
  unsigned long tmin, current = millis();
  do {
    tmin = defaultApprox.time + (n + 0.5) * defaultApprox.velocity + (n * (n+1))/2 * defaultApprox.acceleration;
    n += 12;
  } while(tmin < current + timeDelay);
  defaultApprox.nextDropTime = tmin - timeDelay;
  Serial.println(defaultApprox.acceleration);
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
  defaultApprox.time = 0;
  defaultApprox.nextDropTime = 0;
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
 * Aktualisiere den Wertespeicher.
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
        defaultMemory.hallValue.time = val_millis; // TODO ZEITPUNKT???
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

void addPhotoValue() {
  defaultMemory.photoLastIndex = (defaultMemory.photoLastIndex + 1) % maxNumPhotoValues;
  defaultMemory.photoValues[defaultMemory.photoLastIndex].time = val_millis;
  defaultMemory.photoValues[defaultMemory.photoLastIndex].value = val_photo;
  defaultMemory.photoCount++;
  defaultMemory.photoTotalCount++;
}

/**
 * Diese Prozedur wird verwendet, um das Fallrohr zu öffnen und
 * anschließend für die nächste Kugel vorzubereiten.
 **/
void drop() {
  //eigentliche Prozedur
  myservo.write(17);
  for(pos = 17; pos <= 20; pos += 1) {
    myservo.write(pos);
    busyDelay(20);
  }
  
  //zurücksetzen
  busyDelay(100);
  myservo.write(0);
  busyDelay(100);
  myservo.write(17);
}

/**
 * Verzögerung inklusive Erfassung der Messwerte
 **/
void busyDelay(unsigned long time) {
  unsigned long waitUntil = millis() + time;
  while(millis() < waitUntil) {
    readInputs();
    updateMemory();
  }
}

boolean busyWaitForDrop() {
  unsigned long current;
  while(true) {
    current = millis();
    if(current >= defaultApprox.nextDropTime) {
      return true;
    }
    readInputs();
    if(updateMemory()) {
      approximate();
      if(!defaultApprox.isValid) {Serial.println("bla");
        return false;
      }
      calcDropTime();
    }
  }
}
