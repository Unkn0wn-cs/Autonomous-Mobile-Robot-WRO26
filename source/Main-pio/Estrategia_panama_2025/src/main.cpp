    int LED = 34;

#include <Arduino.h>
#include <AFMotor.h>
#include <QuadratureEncoder.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "C:\Users\samue\OneDrive\Documents\GitHub\singapore25\Codes\Movement\Encoder\Ecoder measured movement\lib\move\move.h"
#include <Pixy2.h>

//move
AF_DCMotor motor1(1); // Motor 1 on the Adafruit Motor Shield
AF_DCMotor motor2(2); // Motor 2 on the Adafruit Motor Shield
AF_DCMotor motor3(3); // Motor 3 on the Adafruit Motor Shield
AF_DCMotor motor4(4); // Motor 4 on the Adafruit Motor Shield
  //LEFT
  int pwmf[4] = {228, 231, 231, 228};
  int pwms[4] = {200, 200, 200, 195};
  // //RIGHT
  // int pwmf[4] = {235, 240, 240, 234};
  // int pwms[4] = {215, 200, 200, 200};
  

Encoders encoderLeft(A15, A14);	// Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders encoderRight(A13 , A12); // Encoder object name rightEncoder using analog pin A0 and A1 
Move move(
  motor1, motor2, motor3, motor4, encoderLeft, encoderRight,
  pwmf[0], pwmf[1], pwmf[2], pwmf[3],      // Forward/backward PWM values
  pwms[0], pwms[1], pwms[2], pwms[3]       // Left/right/diagonal PWM values
);

int startTime;
//define objects----------------------------------------
MPU6050 sensor;
Pixy2 pixy;
Servo myservo;

bool cam;
bool mpu;

//Pixy Cam---------------------------------------------
int purpleSignature = 2;
int orangeSignature = 1;
int redSignature = 3;

const int NUM_FRANJAS = 3;
const int ANCHO_IMAGEN = 316; // Pixy2 horizontal resolution
int pesos[NUM_FRANJAS] = {0};
int camera = 0;

// --- Microswitch Gyro Reset Extension ----------------------------------------------------------------

const byte backSwitchPin = 18; // For interrupt on Mega
const byte sideSwitchPin = 19; // For interrupt on Mega
const byte switchPin = 14; 


volatile bool backSwitchPressed = false;
volatile bool sideSwitchPressed = false;
bool lastBackSwitchState = HIGH;    // for edge detection
bool lastSideSwitchState = HIGH;    // for edge detection

//Rotor variables -------------------------------------------------------------------------
const int enable34 = 9; // L293D pin 9
const int input4   = 46; // L293D pin 10
const int input3   = 48; // L293D pin 15

// Declaración de variables para la velocidad
int velocidad = 210; // Valor inicial de velocidad (0 a 255)
 
// variables for GyroScope ----------------------------------------------------------------------------
// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

//Variables usadas por el filtro pasa bajos
  long f_ax,f_ay, f_az;
  int p_ax, p_ay, p_az;
  long f_gx,f_gy, f_gz;
  int p_gx, p_gy, p_gz;

  //Valor de los offsets
  int ax_o,ay_o,az_o;
  int gx_o,gy_o,gz_o;

  long tiempo_prev;
  float dt;
  float ang_x, ang_y, ang_z; // Added ang_z for Z-axis rotation
  float ang_x_prev, ang_y_prev, ang_z_prev; // Added ang_z_prev for Z-axis rotation
//

//---------------------------------
static int routine = 4;
int state = 0;
bool first = true;

// int closed = 180;
// int open = 0;

int beta = 8; //degree error
int alpha = 0;

int connections;

const int mili = 250; //delay
const int diameter = 60; //Diameter of the wheel in mm

enum rlane {
  OUTER, 
  MIDDLE,
  INNER
}; rlane lane = MIDDLE;

enum side {
  RIGHT,
  LEFT
}; 

//conditional------------------------------------------------------------------------------------- aqui Samuel 
// const long pulses = 1650; // Number of pulses for each movement step
const long pulses = 1020; // Number of pulses for each movement step
// side robotSide = RIGHT;
side robotSide = LEFT;
int lenght = 0;

//servo--------------------------------------------
int closedGate = 170;
int openGate = 55;
//RIGHT
// int closedGate = 96;
// int openGate = 0;

// functions-----------------------------------------------------------------------------

int mm(int mm){
  return(move.mmToPulses(mm, diameter, pulses));
}

bool inner(int mili){
  if (robotSide == RIGHT){
    if(move.left(mm(mili))) return true;
  }else if (robotSide == LEFT){
    if(move.right(mm(mili)))return true;
  }
}

bool outer(int mili){
  if (robotSide == RIGHT){
    if(move.right(mm(mili)))return true;
  }else if (robotSide == LEFT){
    if(move.left(mm(mili))) return true;
  }
}

int testI2C() {
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    } 
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Scan done\n");
  }

  return nDevices; // return how many devices were found
}

int classifyLane(float x, float y, bool right) {
    // Define the two diagonal boundaries
    float mA, mB, bA, bB;
    
    if (right){
      mA = (24.0 - 205.0) / (128.0 - 45.0);  // ≈ -2.346
      bA = 205 - mA * 45;            // ≈ 310.57

      mB = (180.0 - 24.0) / (237.0 - 188.0); // ≈ 2.636
      bB = 24 - mB * 188; 
    } else {
      mA = (61.0 - 182.0) / (144.0 - 6.0);  // ≈ -2.346
      bA = 182 - mA * 6;            // ≈ 310.57

      mB = (186.0 - 8.0) / (279.0 - 161.0); // ≈ 2.636
      bB = 8 - mB * 161;            // ≈ -500.2
    }


    // Evaluate position relative to lines
    float yA = mA * x + bA;
    float yB = mB * x + bB;

    Serial.print("antes de la formula ");
    Serial.println(yA);

    if (y < yA) {
        return 0; // Lane 1
    } else if (y >= yA && y < yB) {
        return 2; // Lane 2
    } else {
        return 1; // Lane 3
    }
}
void enableSlowDrivers(){pinMode(enable34, OUTPUT); digitalWrite(enable34, 50);}
void enableDrivers(){digitalWrite(enable34, 150);   pinMode(enable34, OUTPUT);}
void disableDrivers(){digitalWrite(enable34, 80);   pinMode(enable34, OUTPUT);}

//----------------------------------------------------------------------------------------
int filterGyro(MPU6050_Base sensor){
  sensor.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  ang_z = ang_z_prev + (gz / 131.0) * dt; // Z-axis rotation using gyroscope only

  ang_z_prev = ang_z;
  return ang_z;
} 

void onSwitchPress() {
  backSwitchPressed = true;
  resetGyroAngles();
}

void resetGyroAngles() {
  ang_x = ang_y = ang_z = 0;
  ang_x_prev = ang_y_prev = ang_z_prev = 0;
  tiempo_prev = millis(); 
}

// void waitForSwitchPress() {
//   while (digitalRead(switchPin) == LOW) {
//     // Do nothing — wait until switch is pressed
//   }
// }

  void blink(){
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        delay(500);
        return;
  }

void setup() { //-----------------------------------------------------------------------------------------------------------------------------------///

  Serial.begin(9600);
  
  pixy.init();


  if(robotSide == RIGHT){
    lenght = 680;
  }else{
    lenght = 1100;
  } 
  //servo--------------------------------------------
  myservo.attach(10); 
  
  //rotor
  pinMode(enable34, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);

    // LED DEBUGER SUPERIOR GRAN RESERVA PRO MAX ROJO TRUMP MAGA UNIMET #FORMAFALICA
    pinMode(LED, OUTPUT);

  // //mpu 
  Wire.begin();           // Iniciando I2C
  sensor.initialize();    // Iniciando el sensor

  if (sensor.testConnection()) {Serial.println("Sensor iniciado correctamente"); mpu = true;}
  else{Serial.println("Error al iniciar el sensor"); mpu = false;} 

  tiempo_prev = millis();
  resetGyroAngles();

  //microSwitch
  pinMode(backSwitchPin, INPUT_PULLUP);
  pinMode(sideSwitchPin, INPUT_PULLUP);
  pinMode(switchPin, INPUT_PULLUP);

  if (robotSide == LEFT){
    bool pinpressed = false;
    while(!pinpressed){
      if ((digitalRead(switchPin)) == HIGH){
        pinpressed = true;
      }
    }
  }

  startTime = millis();

  myservo.write(closedGate);
  digitalWrite(input3, HIGH);
  digitalWrite(input4, LOW);
  enableDrivers();


//   //Calculate purple position
int center_y = 32;
int center_x = 200;

int noballs = 0;

for (int i = 0; i < 120; i++){
pixy.ccc.getBlocks();
if (pixy.ccc.numBlocks > 0) {
  for (int j = 0; j < pixy.ccc.numBlocks; j++) {
    if (pixy.ccc.blocks[j].m_signature == purpleSignature &&
        pixy.ccc.blocks[j].m_age > 50
        && pixy.ccc.blocks[j].m_x > 105 && pixy.ccc.blocks[j].m_x < 300 &&
        pixy.ccc.blocks[j].m_y > 20 && pixy.ccc.blocks[j].m_y < 90
        ) {
          
      int ix = pixy.ccc.blocks[j].m_x; 
      int iy = pixy.ccc.blocks[j].m_y;
      //definir routine 
      if (ix < center_x && iy < center_y){
        routine = 0;
        pixy.ccc.blocks[j].print();
        // blink();
        Serial.print("upper left corner, case 1");
        return;
      } else if (ix > center_x && iy < center_y){
        routine = 1;
        pixy.ccc.blocks[j].print();
        // blink(); blink();
        Serial.print("upper right corner, case 3");
        return;
      } else if (ix < center_x && iy > center_y){
        routine = 2;
        pixy.ccc.blocks[j].print();
        // blink(); blink(); blink();
        Serial.print("lower left corner, case 0");
        return;
      } else if (ix > center_x && iy > center_y){
        routine = 3;
        pixy.ccc.blocks[j].print();
        // blink(); blink(); blink(); blink();
        Serial.print("lower Right corner, case 2");
        return;
      }
    } 
  } 
  } else if (noballs < 10){
    noballs++;
  } else {return;}
  delay(14);
}


pixy.setLamp(0, 0);






}

void loop() {//---------------------------------------------------------------------------------------------------------------------------------------------////

  static int microSwitchTime = 0;
  int currentTime = millis();
    bool currentBackSwitchState = digitalRead(backSwitchPin);
    bool currentSideSwitchState = digitalRead(sideSwitchPin);


    if (currentTime - microSwitchTime > 350){
      // Handle switch press
      if (currentBackSwitchState == LOW && lastBackSwitchState == HIGH) {
        microSwitchTime = millis();
        // Button pressed
        onSwitchPress();
        if (!(routine == 6 && state == 2)){
          state++;
        }
        Serial.println("Gyro angles reset by microswitch!");
      }
      if (currentSideSwitchState == LOW && lastSideSwitchState == HIGH && routine!= 4  && !(routine == 7 && state == 8)  ) {
        microSwitchTime = millis();
        // Button pressed
        state++;
      }
      lastBackSwitchState = currentBackSwitchState;
      lastSideSwitchState = currentSideSwitchState;
    }

  //

    // Last Routine Code ----------------------------------------------
    static bool lastRoutine = false;
    if (lastRoutine == false &&  (millis() > 105000 + startTime) && (routine != 7 || routine != 5)){
      lastRoutine = true;
      lane = INNER;
      lenght -= 30;
    }
    static bool midRoutine = false;
    if (midRoutine == false  &&  (millis() > 45000 + startTime) && (routine != 7 || routine != 5)){
      midRoutine = true;
      lane = INNER;
      lenght -= 30;
    } else if(midRoutine == true && (millis() > 62000 + startTime) && (millis() < 100000 + startTime)){
      midRoutine = false;
      lenght += 30;
    }
  int devices = testI2C();

  if (devices > 0) {
    const float GYRO_SENSITIVITY = 131.0; // MPU6050 scale factor for ±250°/s

    // Get filtered Z-axis rotation rate
    float gz_filtered = filterGyro(sensor); // Should return raw gyro Z in deg/s

    // Time delta in seconds
    unsigned long now = millis();
    dt = (now - tiempo_prev) / 1000.0;
    tiempo_prev = now;


    // Integrate angular velocity to get angle
    ang_z = ang_z_prev + (gz_filtered / GYRO_SENSITIVITY) * dt;
    ang_z_prev = ang_z;
  } else {
    Serial.println("I2C FAILED or no devices responding!");

    mpu = false;
  }


    

    




  //check if the robot is centered before acting---------------------------------
  // if (ang_z >= alpha + beta && routine != 7) {
  //   routine = 8;
  // } else if (ang_z <= alpha - beta && routine != 7){
  //   routine = 8;
  // }

    // routine 0-3 = case 0-3 ball 
    // routine 4 Simple Lanes
    // routine 5 Diagonal Lane
    // routine 6 simple reset
    // routine 7 Corner checking reset
    // routine 8 Reorient itse
    // routine 9 Parring
    // routine 10 debugging


switch (routine) {
  case 0:
    switch(state){
      case 0:
        if(move.backward(mm(100))) state++;
        break;
      case 1:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(mm(505))) state++;
        break;
      case 3:
        // if(move.stopForMillis(mili)) 
        state++;
        break;
      case 4:
        myservo.write(openGate);
        if(move.forward(mm(350))) state++;
        break;
      case 5:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 6:
        routine = 6; state = 0;
        if (robotSide == RIGHT){
          lane = OUTER;
        } else {lane = INNER;}
      }
    break;
  case 1:
    switch(state){
      case 0:
        if (move.right(mm(180))) state++;
        break;
      case 1:
        if(move.backward(mm(100))) state++;
        break;
      case 2:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 3:
        if(move.forward(mm(530))) state++;
        break;
      case 4:
        // if(move.stopForMillis(mili))
        state++;
        break;
      case 5:
        myservo.write(openGate);
        if(move.forward(mm(350))) state++;
        break;
      case 6:
        myservo.write(closedGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 7:
        routine = 6; state = 0;
        lane = MIDDLE;
      }
    break;
  case 2:
    switch(state){
      case 0:
        if(move.backward(mm(100))) state++;
        break;
      case 1:
        myservo.write(openGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(mm(550))) state++;
        break;
      case 3:
        // if(move.stopForMillis(mili)) 
        state++;
        break;
      case 4:
        myservo.write(closedGate);
        if(move.forward(mm(250))) state++;
        break;
      case 5:
        routine = 6; state = 0;
        if (robotSide == RIGHT){
          lane = OUTER;
        } else {lane = INNER;}
    }
    break;
  case 3:
    switch(state){
      case 0:
        if(move.right(mm(180))) state++;
        break;
      case 1:
        if(move.backward(mm(100))) state++;
        break;
      case 2:
        myservo.write(openGate);
        if(move.stopForMillis(mili)) state++;
        break;
      case 3:
        if(move.forward(mm(550))) state++;
        break;
      case 4:
        // if(move.stopForMillis(mili)) 
        state++;
        break;
      case 5:
        myservo.write(closedGate);
        if(move.forward(mm(250))) state++;
        break;
      case 6:
        routine = 6; state = 0;
        lane = MIDDLE;
    }
    break;
  





  ////-------------------------------LOOOP-----------------------------------------------------------////
  case 4:
    switch(state){
      case -1:
        if(outer(200)) state++;
        break;
      case 0:
        if(move.backward(mm(280))) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
          enableDrivers();        
          if (lane == MIDDLE){
            if(move.forward(mm(lenght + 50))) state++;  
          } else if(lane == OUTER){
              if (robotSide == RIGHT){
                int test = move.forwardp(mm(lenght + 50), true);
                if (test == 2){enableSlowDrivers();}
                switch(test){
                  case 1:
                    state++;
                    break;
                }

                // if(move.forwardp(mm(lenght + 50), true) == 1) state++;  
              }else{
                int test = move.forwardp(mm(lenght + 50), false);
                if (test == 2){enableSlowDrivers();}
                switch(test){
                  case 1:
                    state++;
                    break;
                }
                
                // if(move.forwardp(mm(lenght + 100), false) == 1) state++;  
              }
          }
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        if(move.stopForMillis(mili)) state++;
        break;
      case 5:

        if (lane == OUTER){
          state = -1;
        } else{state = 0;}

        if (robotSide == RIGHT && first == false){
          if (lane == OUTER){
            lane = MIDDLE;
          }else if(lane == MIDDLE){
            lane = INNER;
          }
        } else if (robotSide == RIGHT && first == true){
          lane = OUTER;
          first = false;
        }else if (robotSide == LEFT){
          first = false;
          if (lane == OUTER){
            lane = MIDDLE;
          } else if (lane == MIDDLE) {
            lane = INNER;
          } else if (lane == INNER) {
            return(1);
          }
        }
        routine = 6;

        if (lastRoutine == true || midRoutine == true){
          routine  = 7;
          state = 0;
        }
      break;
    }
  break;
  case 5:
    enableDrivers();  
    switch(state){
      case 0:
        if(inner(50)) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.backward(600)) state++;
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        if(move.forward(mm(160))) state++;
        break;
      case 5:
        state++;
        break;
      case 6:
        if (robotSide == RIGHT){
          if(move.forwardLeft(mm(280))) state++;
        } else{
          if(move.forwardRight(mm(200))) state++;
        }
        digitalWrite(LED, HIGH); 
        break;
      case 7:
        state++;
        break;
      case 8:
        if (robotSide == RIGHT){
          if(move.forwardq(mm(lenght/3 + 150), true)){state++;}
        } else {
          if(move.forwardq(mm(lenght/2 + 160), false)){state++;}
        }
        break;
      case 9:
        if(move.stopForMillis(2*mili)) state++;
        digitalWrite(LED, LOW);
        break;
      case 10:
        if (robotSide == LEFT){
          if(outer(50)) state++;
        } else{
          state++;
        }
        break;
      case 11:
        if(move.stopForMillis(mili)) state++;
        break;
      case 12:
        lane = OUTER;
        routine = 7;
        state = 0;
        break;
    }
    break;
  case 6:
    switch(state){
      case -1:
        if (robotSide == RIGHT){
          if(move.backwardp(mm(lenght + 50), true)) state = 1;  
        }else{
          if(move.backwardp(mm(lenght + 100), false)) state = 1;  
        }
        break;
      case 0:
        if(move.backward(mm(lenght + 250))) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(outer(750)) state++;
        break;
      case 3:
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        static bool first = true;
        if (!(lane == OUTER) && first == true){
          if(inner(180)) state++;
        } else if (first == true){
          state = 7;
          first = false;
          break;
        }else if(!first){
          if(inner(180)) state++;
        }
        pixy.setLamp(0, 0);
        break;
      case 5:
        if(move.backward(mm(200))) state++;
        break;
      case 6:

        pixy.ccc.getBlocks();

        if (pixy.ccc.numBlocks > 0) {
          for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == orangeSignature){
            Block block = pixy.ccc.blocks[i];

            // Clasifica en franja según posición X
            int franja = classifyLane(block.m_x, block.m_y, true);
            franja = constrain(franja, 0, NUM_FRANJAS - 1);

            // Calcula tamaño como área
            int tamano = block.m_width * block.m_height;

            // Suma al peso de la franja
            pesos[franja] += tamano;
          }
        }
          camera++;
        } else {
          camera = 0;
        }

        if(move.stopForMillis(mili/2)) {
          // Encuentra la franja con mayor peso
            int mejorFranja = 0;
            for (int i = 1; i < NUM_FRANJAS; i++) {
              if (pesos[i] > pesos[mejorFranja]) {
                mejorFranja = i;
              }
            }

            if (connections < 4){
              if (mejorFranja == 0){
                if (robotSide == RIGHT){
                  lane = INNER;
                } else {lane == OUTER;}
              } else if(mejorFranja == 2){
                if(robotSide == RIGHT){
                  lane = OUTER;
                } else {lane = INNER;}
              } else if (mejorFranja == 1){
                  lane = MIDDLE;
              }
              connections++;
            }
          
          state++;
          pixy.setLamp(0, 0);
        }

        


        break;
          
      case 7: 
          //Reset Camera weights
          for (int i = 0; i < NUM_FRANJAS; i++) {
            pesos[i] = 0;
          }

          if(lane == INNER){
            routine = 5;
            state = 0;
          }else if(lane == MIDDLE){
            routine = 4;
            state = 0;
          }else if(lane == OUTER){
            routine = 4;
            state = -1;
        }

        if (lastRoutine == true){
          routine = 5;
          state = 0;
        } else if(midRoutine == true){
          routine = 7;
          state = 0;
        }
      break;

        
    }
  break;
  case 7:
    connections = 0;
    switch(state){
      case 0:
        if(move.backward(mm(lenght + 250))) state++;
        break;
      case 1:
        if(move.stopForMillis(mili)) state++;
        break;
      case 2:
        if(move.forward(80)) state++;
        break;
      case 3:
        digitalWrite(enable34, LOW);
        if(move.stopForMillis(mili)) state++;
        break;
      case 4:
        digitalWrite(LED, HIGH);

        if (mpu == true){
          if (robotSide == LEFT){
            alpha = -80;
          } else {alpha  = 80;}
          
          if (ang_z >= alpha + beta) {
              move.rotateCCW(200, 200, 200, 200);
          } else if (ang_z <= alpha - beta){
              move.rotateCW(200, 200, 200, 200);
          } else {
            state++;
          } 
        } else{
          if(robotSide == RIGHT){
            if(move.rotate(mm(166), false)) state++;
          } else {
            if(move.rotate(mm(166), true)) state++;
          }
        }

        break;
      case 5:
        if(move.stopForMillis(mili)) state++;
        digitalWrite(LED, LOW);
        break;
      case 6:
        if(outer(mm(20))) state++;
        break;
      case 7:
        if(move.stopForMillis(mili/2)) state++;
        break;
      case 8:
          if (robotSide == RIGHT){
            if(move.forwardp(mm(550), true) == 1) state++;  
          }else{
            if(move.forwardp(mm(550), false) == 1) state++;  
          }
        break;
      case 9:
        enableDrivers();
        if(move.stopForMillis(250)) state++;
        break;
      case 10:
        digitalWrite(enable34, LOW);
        if(move.backward(20)) state++;
        break;
      case 11:
        if(move.stopForMillis(mili/2)) state++;
        // rotation = false;
        break;
      case 12:
        digitalWrite(LED, HIGH);

        if (mpu == true){
          if (robotSide == LEFT){
            alpha = -80;
          } else {alpha  = 80;}
          if (ang_z >= alpha + beta) {
              move.rotateCCW(200, 200, 200, 200);
          } else if (ang_z <= alpha - beta){
              move.rotateCW(200, 200, 200, 200);
          } else {
            state++;
          } 
        } else{
          if(robotSide != RIGHT){
            if(move.rotate(mm(166), false)) state++;
          } else {
            if(move.rotate(mm(166), true)) state++;
          }
        }
        break;
      case 13:
        if(move.stopForMillis(mili)) state++;
        digitalWrite(LED, LOW);
        break;
      case 14:
        if(outer(120)) state++;
        break;
      case 15:
        if(move.stopForMillis(mili)) state++;
        break;
      case 16:
        enableDrivers();
        if(robotSide == RIGHT){
          if(move.rotate(mm(50), true)) state++;
        } else {
          if(move.rotate(mm(50), false)) state++;
        }
        break;
      case 17:
        if(move.stopForMillis(mili * 2)) state++;
        break;
      case 18:
        if(robotSide == LEFT){
          if(move.rotate(mm(25), true)) state++;
        } else {
          if(move.rotate(mm(25), false)) state++;
        }
        break; 
      case 19:
        if(move.stopForMillis(mili)) state++;
        break; 
      case 20:
        routine = 4;
        state = 0;
        lane = OUTER;
    }
  break;
  case 8:
    alpha = 0;
    if (ang_z >= alpha + beta) {
        move.rotateCCW(200, 200, 200, 200);
    } else if (ang_z <= alpha - beta) {
        move.rotateCW(200, 200, 200, 200);
    } else {
        routine = 7;
        state = 0;
    }

  case 9:
    // if (lastRoutine == false and millis() > 62000){
    //   routine = 6;
    //   state = 0;
    // }
    move.moveBeginStrafe();
    pixy.ccc.getBlocks();
    if (robotSide == LEFT){
      if (pixy.ccc.numBlocks) {
        int maxArea = 0;
        int maxIndex = -1;
        int moveby =  40;
        int maxmove = 550;
        static int movement = 0;

        // Step 1: Find the largest orange block
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
          if (pixy.ccc.blocks[i].m_signature == orangeSignature) {
            int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
            if (area > maxArea) {
              maxArea = area;
              maxIndex = i;
            }
          }
        }

        if (maxIndex != -1) {
          int x = pixy.ccc.blocks[maxIndex].m_x;

          if (x > 240 &&
            ((robotSide == RIGHT && movement > 0) ||
            (robotSide == LEFT && movement < maxmove))) {

            while (true){
              if(move.right(mm(moveby))) break;
            }
            Serial.print("MOVE RIGHT\n");
            pixy.ccc.blocks[maxIndex].print();
            if (robotSide == RIGHT){
              movement -= moveby;
            } else {
              movement += moveby;
            }
          } else if (x < 140 &&
            ((robotSide == LEFT && movement > 0) ||
            (robotSide == RIGHT && movement < maxmove))) {
            
            while (true){
              if(move.left(mm(moveby))) break;
            }
            Serial.print("MOVE LEFT\n");
            pixy.ccc.blocks[maxIndex].print();
            if (robotSide == RIGHT){
              movement += moveby;
            } else {
              movement -= moveby;
            }
          } else {
            move.stop();
            pixy.ccc.blocks[maxIndex].print();
          }
        }
      } delay(5);
    }else {
      routine = 10;
      state = 0;
    }
    if (lastRoutine == false and millis() > 62000){
      routine = 6;
      state = 0;
    }   
    break;

  case 10:
    switch(state){
      case 0:
        if(move.backward(mm(15))) state++;
        break;
      case 1:
        if(move.left(mm(500))) state++;
        break;
    }
    break;
  
} 



  delay(5); //delay for stability introduced july 😝
}
