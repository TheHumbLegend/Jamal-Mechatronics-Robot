#include <Servo.h>
//  Servos 
Servo S1; // Base
Servo S2; // Elbow
Servo S3; // Gripper
const byte S1pwm = 34 ;
const byte S2pwm = 40;
const byte S3pwm = 52;
//  Robot Dimensions (mm) 
float L1 = 81;    // upper arm
float L2 = 282;   // forearm

// Motor Driver Pins
#define PWMA 5
#define PWMB 6

#define AI1 22
#define AI2 23

#define BI1 24
#define BI2 25

// Battery Indicator LEDs
#define LED1 11
#define LED2 12
#define LED3 13

// Button
#define BUTTON 4

// Battery voltage reading
#define BATTERY_PIN A0

// Encoder pins
#define EncoderLEFT_A  2
#define EncoderLEFT_B  3
#define EncoderRIGHT_A 18
#define EncoderRIGHT_B 19

//IR Sensor
#define IR_SENSOR_PIN A1
const float STOP_DISTANCE_CM = 26.5; 


// VARIABLES
volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

bool motorsRunning   = false;
bool lastButtonState = LOW;

const float R1 = 10000.0;
const float R2 = 10000.0;

//Max speed
const int MAX_SPEED = 100;
float straightKp   = 1.5;
float straightKi   = 0.05;
float straightIPrev = 0;;

// PI Controller variables — forward motion
float encKp    = 2.0;
float encKi    = 0.1;
float encIPrev = 0;
float encError = 0;
float refPosition = 0;
float position    = 0;

// PI Controller variables — turning
float turnKp    = 2.0;
float turnKi    = 0.1;
float turnIPrev = 0;
float turnError = 0;
float refAngle  = 0;
float angle     = 0;

unsigned long lastTime       = 0;
unsigned long lastDisplay    = 0;
unsigned long stateStartTime = 0;

const float wheelDiameter      = 6.0;//prev 6.3
const float wheelBase          = 13.5;//15.2 prev
const float wheelCircumference = 3.1416 * wheelDiameter;
const float ENC_K              = 3840; // Full quadrature: 960 x 4

int state = 0;


// ENCODER INTERRUPTS full quad

void leftEncoderISR_A() {
    if (digitalRead(EncoderLEFT_A) == HIGH) {
        if (digitalRead(EncoderLEFT_B) == HIGH) leftEncoderCount++;
        else leftEncoderCount--;
    } else {
        if (digitalRead(EncoderLEFT_B) == LOW) leftEncoderCount++;
        else leftEncoderCount--;
    }
}

void leftEncoderISR_B() {
    if (digitalRead(EncoderLEFT_B) == HIGH) {
        if (digitalRead(EncoderLEFT_A) == LOW) leftEncoderCount++;
        else leftEncoderCount--;
    } else {
        if (digitalRead(EncoderLEFT_A) == HIGH) leftEncoderCount++;
        else leftEncoderCount--;
    }
}

void rightEncoderISR_A() {
    if (digitalRead(EncoderRIGHT_A) == HIGH) {
        if (digitalRead(EncoderRIGHT_B) == HIGH) rightEncoderCount++;
        else rightEncoderCount--;
    } else {
        if (digitalRead(EncoderRIGHT_B) == LOW) rightEncoderCount++;
        else rightEncoderCount--;
    }
}

void rightEncoderISR_B() {
    if (digitalRead(EncoderRIGHT_B) == HIGH) {
        if (digitalRead(EncoderRIGHT_A) == LOW) rightEncoderCount++;
        else rightEncoderCount--;
    } else {
        if (digitalRead(EncoderRIGHT_A) == HIGH) rightEncoderCount++;
        else rightEncoderCount--;
    }
}

// Safe atomic read of volatile long on AVR
long readEncoder(volatile long &enc) { //by ref
    noInterrupts();
    long val = enc;
    interrupts();
    return val;
}

int moveServo(Servo &servo, const char* servoName, int step, int refPulseWidth, float servoRadAngle, float increment = 5.0) {

  const float Pi = 3.142;

  float minPulseWidth = float(refPulseWidth) - 1000.0;  
  int targetPWM = (servoRadAngle + (Pi/2)) * (2000.0 / Pi) + minPulseWidth;

  int currentPWM = servo.readMicroseconds();

  while (currentPWM != targetPWM) {
    if (targetPWM > currentPWM) {
      currentPWM = min(targetPWM, currentPWM + (int)increment);
    } else if (targetPWM < currentPWM) {
      currentPWM = max(targetPWM, currentPWM - (int)increment);
    }
    servo.writeMicroseconds(currentPWM);
    delay(15);
  }

  float angleDeg = servoRadAngle * 180.0 / Pi;

  return targetPWM;
}

// SETUP
void setup() {
    Serial.begin(9600);

    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(AI1,  OUTPUT);
    pinMode(AI2,  OUTPUT);
    pinMode(BI1,  OUTPUT);
    pinMode(BI2,  OUTPUT);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);

    pinMode(BUTTON, INPUT); // External pull-down resistor on hardware

    pinMode(EncoderLEFT_A,  INPUT);
    pinMode(EncoderLEFT_B,  INPUT);
    pinMode(EncoderRIGHT_A, INPUT);
    pinMode(EncoderRIGHT_B, INPUT);

    pinMode(IR_SENSOR_PIN, INPUT);

    // Full quadrature — attach interrupts to both A and B channels
    attachInterrupt(digitalPinToInterrupt(EncoderLEFT_A),  leftEncoderISR_A,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncoderLEFT_B),  leftEncoderISR_B,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncoderRIGHT_A), rightEncoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncoderRIGHT_B), rightEncoderISR_B, CHANGE);

    S1.attach(S1pwm);
    S2.attach(S2pwm);
    S3.attach(S3pwm);

    stopMotors();

    // Immediately set servos to starting position 
    moveServo(S1, "Base", 0, 1466, 1.22);
    moveServo(S2, "Elbow", 0, 1500, -1.57);
    moveServo(S3, "Gripper", 0, 1504, 1.57);
    Serial.println("System Ready. Press button to start motors.");
}


// BATTERY FUNCTIONS
float readBatteryVoltage() {
    int raw = analogRead(BATTERY_PIN);
    float voltage = (raw / 1023.0) * 5.0;
    voltage = voltage * ((R1 + R2) / R2);
    return voltage;
}

void updateBatteryLEDs(float voltage) {
    if (voltage >= 7.2) {
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, HIGH);
    } else if (voltage >= 7) {
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, HIGH);
    } else if (voltage >= 6.8) {
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, HIGH);
    } else {
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
    }
}


// MOTOR FUNCTIONS
void setMotors(int leftPWM, int rightPWM) {
    if (leftPWM > 0) {
        digitalWrite(AI1, HIGH); digitalWrite(AI2, LOW);
    } else if (leftPWM < 0) {
        digitalWrite(AI1, LOW);  digitalWrite(AI2, HIGH);
    } else {
        digitalWrite(AI1, LOW);  digitalWrite(AI2, LOW);
    }

    if (rightPWM > 0) {
        digitalWrite(BI1, HIGH); digitalWrite(BI2, LOW);
    } else if (rightPWM < 0) {
        digitalWrite(BI1, LOW);  digitalWrite(BI2, HIGH);
    } else {
        digitalWrite(BI1, LOW);  digitalWrite(BI2, LOW);
    }

    analogWrite(PWMA, abs(leftPWM));
    analogWrite(PWMB, abs(rightPWM));
}

void stopMotors() {
    setMotors(0, 0);
}

float readIRDistance() {
    // Average 5 readings to filter noise
    float sum = 0;
    for (int i = 0; i < 5; i++) {
        int raw = analogRead(IR_SENSOR_PIN);
        float voltage = (raw / 1023.0) * 5.0;
        sum += (27.86 / voltage) - 0.42;
        delay(2);
    }
    return sum / 5.0;
}

// BUTTON TOGGLE 
void checkButton() {
    bool currentButtonState = digitalRead(BUTTON);

    
    if (lastButtonState == LOW && currentButtonState == HIGH) {
        motorsRunning = !motorsRunning;
        Serial.println(motorsRunning ? "Motors STARTED" : "Motors STOPPED");
        delay(200); // debounce
    }

    lastButtonState = currentButtonState;
}


// PI CONTROLLER
float piController(float error, float Kp, float Ki, float deltaT, float &iPrev) {
    float p = Kp * error;
    float i = iPrev + (Ki * error * deltaT);
    if (i >  150) i =  150;
    if (i < -150) i = -150;
    float m = p + i;
    if (m >  MAX_SPEED) m = MAX_SPEED;
    if (m < -MAX_SPEED) m = -MAX_SPEED;
    iPrev = i;
    return m;
}

// Inverse Kinematics 
void inverseKinematics(float x, float z_rel, float L1, float L2, float &theta2, float &theta3) {
    float r = sqrt(x*x + z_rel*z_rel);
    theta2 = (atan2(z_rel, x) + acos((L1*L1 + r*r - L2*L2) / (2 * L1 * r)))-1.57;
    theta3 = (PI - acos((L1*L1 + L2*L2 - r*r) / (2 * L1 * L2)))-1.57;
}


// MAIN LOOP
void loop() {
    unsigned long now = millis();

    float voltage = readBatteryVoltage();
    updateBatteryLEDs(voltage);

switch (state) {

    case 0: // Wait for button press
        checkButton();
        if (motorsRunning) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            turnIPrev = 0;
            angle     = 0;
            position  = 0;
            refAngle  = 405.0;
            lastTime  = now;
            stateStartTime = now;
            state = 1;
        }
        break;

    case 1: // Turn right 360 deg
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            float dTheta = ((leftRevs - rightRevs) * wheelCircumference) / wheelBase;
            angle     = dTheta * (180.0 / 3.1416);
            turnError = refAngle - angle;

            float ctrl = piController(turnError, turnKp, turnKi, deltaT, turnIPrev);

            if (abs(ctrl) < 50 && abs(turnError) > 2.0)
                ctrl = (ctrl > 0) ? 50 : -50;

            setMotors(-(int)ctrl, (int)ctrl);

            if (abs(turnError) < 2.0) {
                stopMotors();
                stateStartTime = now;
                state = 99;
            }
        }
        break;

    case 99: // Settle after 360
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            turnIPrev = 0;
            refAngle  = 45.0;
            lastTime  = now;
            state = 2;
        }
        break;

    case 2: // Turn right 45 deg
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            float dTheta = ((leftRevs - rightRevs) * wheelCircumference) / wheelBase;
            angle     = dTheta * (180.0 / 3.1416);
            turnError = refAngle - angle;

            float ctrl = piController(turnError, turnKp, turnKi, deltaT, turnIPrev);

            if (abs(ctrl) < 50 && abs(turnError) > 0.1)
                ctrl = (ctrl > 0) ? 50 : -50;

            setMotors(-(int)ctrl, (int)ctrl);

            if (abs(turnError) < 2.0) {
                stopMotors();
                stateStartTime = now;
                state = 98;
            }
        }
        break;

    case 98: // Settle after right 45 — reset for forward
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            encIPrev       = 0;
            straightIPrev  = 0;   //-- reset straight correction
            refPosition    = 14.0;
            lastTime       = now;
            state = 3;
        }
        break;

    case 3: // Forward 11cm
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            position  = ((leftRevs + rightRevs) / 2.0) * wheelCircumference;
            encError  = refPosition - position;

            float ctrl = piController(encError, encKp, encKi, deltaT, encIPrev);

            if (abs(ctrl) < 50 && abs(encError) > 0.1)
                ctrl = (ctrl > 0) ? 50 : -50;

            // Straight-line correction
            float encDiff = (float)(rightSnap - leftSnap);
            float straightCorr = straightKp * encDiff;
            straightIPrev += straightKi * encDiff * deltaT;
            straightIPrev  = constrain(straightIPrev, -30, 30);
            straightCorr  += straightIPrev;
            straightCorr   = constrain(straightCorr, -30, 30);

            setMotors((int)ctrl - (int)straightCorr, (int)ctrl + (int)straightCorr);

            if (abs(encError) < 0.1) {
                stopMotors();
                stateStartTime = now;
                state = 97;
            }
        }
        break;

    case 97: // Settle after forward — reset for turn
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            turnIPrev = 0;
            refAngle  = -45.0;
            lastTime  = now;
            state = 4;
        }
        break;

    case 4: // Turn left 45 deg
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            float dTheta = ((leftRevs - rightRevs) * wheelCircumference) / wheelBase;
            angle     = dTheta * (180.0 / 3.1416);
            turnError = refAngle - angle;

            float ctrl = piController(turnError, turnKp, turnKi, deltaT, turnIPrev);

            if (abs(ctrl) < 50 && abs(turnError) > 0.1)
                ctrl = (ctrl > 0) ? 50 : -50;

            setMotors(-(int)ctrl, (int)ctrl);

            if (abs(turnError) < 2.0) {
                stopMotors();
                stateStartTime = now;
                state = 96;
            }
        }
        break;

    case 96: // Settle after left 45 — reset for forward
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            encIPrev       = 0;
            straightIPrev  = 0;   // <-- reset straight correction
            refPosition    = 50.0;
            lastTime       = now;
            state = 5;
        }
        break;

    case 5: // Forward 51cm
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            position  = ((leftRevs + rightRevs) / 2.0) * wheelCircumference;
            encError  = refPosition - position;

            float ctrl = piController(encError, encKp, encKi, deltaT, encIPrev);

            if (abs(ctrl) < 50 && abs(encError) > 0.1)
                ctrl = (ctrl > 0) ? 50 : -50;

            // Straight-line correction
            float encDiff = (float)(rightSnap - leftSnap);
            float straightCorr = straightKp * encDiff;
            straightIPrev += straightKi * encDiff * deltaT;
            straightIPrev  = constrain(straightIPrev, -30, 30);
            straightCorr  += straightIPrev;
            straightCorr   = constrain(straightCorr, -30, 30);

            setMotors((int)ctrl - (int)straightCorr, (int)ctrl + (int)straightCorr);

            if (abs(encError) < 0.1) {
                stopMotors();
                stateStartTime = now;
                state = 95;
            }
        }
        break;

    case 95: // Settle after forward — reset for turn
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            turnIPrev = 0;
            refAngle  = -101.0;
            lastTime  = now;
            state = 6;
        }
        break;

    case 6: // Turn left 90 deg
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            float dTheta = ((leftRevs - rightRevs) * wheelCircumference) / wheelBase;
            angle     = dTheta * (180.0 / 3.1416);
            turnError = refAngle - angle;

            float ctrl = piController(turnError, turnKp, turnKi, deltaT, turnIPrev);

            if (abs(ctrl) < 50 && abs(turnError) > 0.1)
                ctrl = (ctrl > 0) ? 50 : -50;

            setMotors(-(int)ctrl, (int)ctrl);

            if (abs(turnError) < 2.0) {
                stopMotors();
                stateStartTime = now;
                state = 94;
            }
        }
        break;

    case 94: // Settle after left 90 — reset for forward
         if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            encIPrev      = 0;
            straightIPrev = 0;
            lastTime      = now;
            state = 7;
    }
        break;

    case 7: // Forward untill IR detects whiteboard
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            // Straight-line correction
            float encDiff = (float)(rightSnap - leftSnap);
            float straightCorr = 0.5 * encDiff;
            straightCorr   = constrain(straightCorr, -15, 15);

            setMotors(90 - (int)straightCorr, 90 + (int)straightCorr);

            float dist = readIRDistance();
            Serial.print("IR Distance: "); Serial.println(dist); 

            if (dist <= STOP_DISTANCE_CM) {
                stopMotors();
                stateStartTime = now;
                state = 93;
        }
        }
        break;

    case 93: // Settle after forward 85.5
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            state = 8;
        }
        break;

    case 8: { //  SERVO ACTION 1 
        // TODO: trigger arm movement and drawing
        float x = 235;  // fixed horizontal position
        float z_values[15] = {230, 220, 210, 200, 190, 180, 170, 160, 150, 140, 130, 120, 110, 100, 90};

         delay(1000);  // brief pause before starting movement
  
        //Begin straight-line movement 
        Serial.println("Beginning IK straight-line sequence...");
        for (int i = 0; i < 15; i++) {
            float z_rel = z_values[i];
            float theta2, theta3;
            inverseKinematics(x, z_rel, L1, L2, theta2, theta3);
            moveServo(S1, "Base", i, 1500, theta2);
            moveServo(S2, "Elbow", i, 1504, theta3);;
            delay(1000);
        }
        
        moveServo(S1, "Base", 0, 1466, 1.22);
        moveServo(S2, "Elbow", 0, 1500, -1.57);
        state = 92;
        break;
    }
    case 92: // Settle after servo action 1 — reset for turn
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            turnIPrev = 0;
            refAngle  = -95.0;
            lastTime  = now;
            state = 9;
        }
        break;

    case 9: // Turn left 90 deg
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            float dTheta = ((leftRevs - rightRevs) * wheelCircumference) / wheelBase;
            angle     = dTheta * (180.0 / 3.1416);
            turnError = refAngle - angle;

            float ctrl = piController(turnError, turnKp, turnKi, deltaT, turnIPrev);

            if (abs(ctrl) < 50 && abs(turnError) > 0.1)
                ctrl = (ctrl > 0) ? 50 : -50;

            setMotors(-(int)ctrl, (int)ctrl);

            if (abs(turnError) < 2.0) {
                stopMotors();
                stateStartTime = now;
                state = 91;
            }
        }
        break;

    case 91: // Settle after left 90 — reset for forward
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            encIPrev       = 0;
            straightIPrev  = 0;   // <-- reset straight correction
            refPosition    = 66.0;
            lastTime       = now;
            state = 10;
        }
        break;

    case 10: // Forward 70cm
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            position  = ((leftRevs + rightRevs) / 2.0) * wheelCircumference;
            encError  = refPosition - position;

            float ctrl = piController(encError, encKp, encKi, deltaT, encIPrev);

            if (abs(ctrl) < 50 && abs(encError) > 0.1)
                ctrl = (ctrl > 0) ? 50 : -50;

            // Straight-line correction
            float encDiff = (float)(rightSnap - leftSnap);
            float straightCorr = straightKp * encDiff;
            straightIPrev += straightKi * encDiff * deltaT;
            straightIPrev  = constrain(straightIPrev, -30, 30);
            straightCorr  += straightIPrev;
            straightCorr   = constrain(straightCorr, -30, 30);

            setMotors((int)ctrl - (int)straightCorr, (int)ctrl + (int)straightCorr);

            if (abs(encError) < 0.1) {
                stopMotors();
                stateStartTime = now;
                state = 90;
            }
        }
        break;

    case 90: // Settle after forward 70 — reset for turn
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            turnIPrev = 0;
            refAngle  = -100.0;
            lastTime  = now;
            state = 11;
        }
        break;

    case 11: // Turn left 90 deg
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            float dTheta = ((leftRevs - rightRevs) * wheelCircumference) / wheelBase;
            angle     = dTheta * (180.0 / 3.1416);
            turnError = refAngle - angle;

            float ctrl = piController(turnError, turnKp, turnKi, deltaT, turnIPrev);

            if (abs(ctrl) < 50 && abs(turnError) > 0.1)
                ctrl = (ctrl > 0) ? 50 : -50;

            setMotors(-(int)ctrl, (int)ctrl);

            if (abs(turnError) < 2.0) {
                stopMotors();
                stateStartTime = now;
                state = 89;
            }
        }
        break;

    case 89: // Settle after left 90 — reset for forward
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            encIPrev       = 0;
            straightIPrev  = 0;   // <-- reset straight correction
            refPosition    = 58.0;
            lastTime       = now;
            state = 12;
        }
        break;

    case 12: // Forward 60cm
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            position  = ((leftRevs + rightRevs) / 2.0) * wheelCircumference;
            encError  = refPosition - position;

            float ctrl = piController(encError, encKp, encKi, deltaT, encIPrev);

            if (abs(ctrl) < 50 && abs(encError) > 0.1)
                ctrl = (ctrl > 0) ? 50 : -50;

            // Straight-line correction
            float encDiff = (float)(rightSnap - leftSnap);
            float straightCorr = straightKp * encDiff;
            straightIPrev += straightKi * encDiff * deltaT;
            straightIPrev  = constrain(straightIPrev, -30, 30);
            straightCorr  += straightIPrev;
            straightCorr   = constrain(straightCorr, -30, 30);

            setMotors((int)ctrl - (int)straightCorr, (int)ctrl + (int)straightCorr);

            if (abs(encError) < 0.1) {
                stopMotors();
                stateStartTime = now;
                state = 88;
            }
        }
        break;
    
        case 88: // Settle after forward 70 — reset for turn
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            turnIPrev = 0;
            refAngle  = -95.0;
            lastTime  = now;
            state = 13;
        }
        break;

    case 13: // Turn left 90 deg
        if (now - lastTime >= 10) {
            float deltaT = (now - lastTime) / 1000.0;
            lastTime = now;

            long leftSnap  = readEncoder(leftEncoderCount);
            long rightSnap = readEncoder(rightEncoderCount);

            float leftRevs  = (float)leftSnap  / ENC_K;
            float rightRevs = (float)rightSnap / ENC_K;
            float dTheta = ((leftRevs - rightRevs) * wheelCircumference) / wheelBase;
            angle     = dTheta * (180.0 / 3.1416);
            turnError = refAngle - angle;

            float ctrl = piController(turnError, turnKp, turnKi, deltaT, turnIPrev);

            if (abs(ctrl) < 50 && abs(turnError) > 0.1)
                ctrl = (ctrl > 0) ? 50 : -50;

            setMotors(-(int)ctrl, (int)ctrl);

            if (abs(turnError) < 2.0) {
                stopMotors();
                stateStartTime = now;
                state = 87;
            }
        }
        break;

    case 87: // Settle after forward 60
        if (now - stateStartTime >= 600) {
            noInterrupts();
            leftEncoderCount  = 0;
            rightEncoderCount = 0;
            interrupts();
            state = 14;
        }
        break;


    case 14: // SERVO ACTION 2 
        // TODO: trigger arm movement and drawing
        moveServo(S1, "Base", 0, 1466, -1.25); //-1.46
        moveServo(S2, "Elbow", 0, 1500, -1.57);
        delay(3000);
        S3.writeMicroseconds(1504);
        state = 15;
        break;

    case 15: // Sequence complete
        stopMotors();
        motorsRunning = false;
        state = 0;
        Serial.println("Sequence complete. Press button to run again.");
        break;
}


    // Print status every 250 ms
    if (now - lastDisplay >= 250) {
        lastDisplay = now;
        Serial.print("State: ");        Serial.print(state);
        Serial.print("   Pos: ");       Serial.print(position, 2);
        Serial.print(" cm   Angle: ");  Serial.print(angle, 2);
        Serial.print(" deg   Left Enc: ");  Serial.print(readEncoder(leftEncoderCount));
        Serial.print("   Right Enc: ");     Serial.print(readEncoder(rightEncoderCount));
        Serial.print("   Voltage: ");       Serial.println(voltage, 2);
    }
}


