/**
 * Implementierung: Kugelfall
 *
 * @author Chris Köcher
 * @author Philipp Schlag
 */

#include <Servo.h> 
Servo myservo;

const int maxNumPhotoValues = 7;
const int timeDelay = 391 + 2*42; //TODO!!!
const int readDelay = 42; //TODO!!!
const int approxDelay = 42; //TODO!!!
const int calcDelay = 42; //TODO!!!

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
  int time;
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
 * Scheibenposition, Geschwindigkeit und Verzögerung approximieren
 **/
void approximate() {
  // überprüfe, ob genügend aktuelle Photowerte vorliegen
  if(defaultMemory.photoTotalCount < maxNumPhotoValues) {
    defaultApprox.isValid = false;
    digitalWrite(pin_out_led2, LOW);
    return;
  }
  
  // überprüfe, ob die Beschleunigung in Ordnung ist
  float t1, t2, quot;
  t2 = millis() - defaultMemory.photoValues[defaultMemory.photoLastIndex].time;
  t1 = defaultMemory.photoValues[defaultMemory.photoLastIndex].time - defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-1)%maxNumPhotoValues].time;
  quot = t2/t1;
  if(quot > 1.2) {
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
    if(quot < 0.8 || quot > 1.2) {
      defaultApprox.isValid = false;
      digitalWrite(pin_out_led2, LOW);
      return;
    }
  }
  
  // Berechnung des Winkels
  defaultApprox.angle = 15 + ((int)defaultMemory.hallValue.value)*180 + (defaultMemory.photoCount-1)*30;
  
  
  // Geschwindigkeit
  defaultApprox.velocity = (defaultMemory.photoValues[defaultMemory.photoLastIndex].time - defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-6)%maxNumPhotoValues].time) / 6;
  
  // Geschwindigkeit im gültigen Bereich?
  if(defaultApprox.velocity < 20.83 || defaultApprox.velocity > 833.33) {
    defaultApprox.isValid = false;
    digitalWrite(pin_out_led2, LOW);
    return;
  }
  
  // Verzögerung
  defaultApprox.acceleration = ((float)(defaultMemory.photoValues[defaultMemory.photoLastIndex].time - 2*defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-3)%maxNumPhotoValues].time + defaultMemory.photoValues[(defaultMemory.photoLastIndex+maxNumPhotoValues-6)%maxNumPhotoValues].time)) / 3;
  
  // fertig
  defaultApprox.time = defaultMemory.photoValues[defaultMemory.photoLastIndex].time;
  defaultApprox.isValid = true;
  digitalWrite(pin_out_led2, HIGH);
  
}

/**
 * Berechne die nächstmögliche Fallzeit
 **/
void calcDropTime() {
  int n = (12 - (defaultApprox.angle + 15) / 30) % 12;
  int tmin, current = millis();
  do {
    tmin = defaultApprox.time + (n + 0.5) * defaultApprox.velocity + (n * (n+1))/2 * defaultApprox.acceleration;
    n += 12;
  } while(tmin < current + timeDelay);
  
  defaultApprox.nextDropTime = tmin - timeDelay;
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
  defaultApprox.time = -1;
  defaultApprox.nextDropTime = -1;
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
    defaultMemory.photoValues[i].time = -1;
    //defaultMemory.photoValues[i].value = -1.0;
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
      if(val_photo != (int)defaultMemory.photoValues[defaultMemory.photoLastIndex].value) {
        addPhotoValue();
        newPhotoValue = true;
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
        newPhotoValue = true;
      }
      break;
    case READY:
      if(val_photo != (int)defaultMemory.photoValues[defaultMemory.photoLastIndex].value) {
        addPhotoValue();
        newPhotoValue = true;
      }
      if(val_hall != (int)defaultMemory.hallValue.value) {
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
  time_start = millis();//debugging
  
  //eigentliche Prozedur
  myservo.write(17);
  for(pos = 17; pos <= 20; pos += 1) {
    myservo.write(pos);
    busyDelay(20);
  }
  
  //debugging
  time_end = millis();
  time_diff = time_end - time_start;
  Serial.println(time_diff);
  
  //zurücksetzen
  busyDelay(50);
  myservo.write(0);
  busyDelay(50);
  myservo.write(17);
}

/**
 * Verzögerung inklusive Erfassung der Messwerte
 **/
void busyDelay(int time) {
  int waitUntil = millis() + time;
  while(millis() < waitUntil) {
    if(millis() + readDelay < waitUntil) {
  	  readInputs();
      updateMemory();
    }
  }
}

boolean busyWaitForDrop() {
  int current;
  while(true) {
    current = millis();
    if(current >= defaultApprox.nextDropTime) {
      return true;
    }
    current += readDelay;
    if(current < defaultApprox.nextDropTime) {
      readInputs();
      updateMemory();
      current += approxDelay;
      if(current < defaultApprox.nextDropTime) {
        approximate();
        if(!defaultApprox.isValid) {
          return false;
        }
        if(current + calcDelay < defaultApprox.nextDropTime) {
          calcDropTime();
        }
      }
    }
  }
}
