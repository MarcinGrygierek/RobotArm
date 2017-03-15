#include <StandardCplusplus.h>
#include <serstream>
#include <string>
#include <vector>
#include <iterator>

#include "Servo.h"             //Servo library
#include "Math.h"

using namespace std;

/* CONSTANTS */
const int SERVOS_COUNT = 7; //ilość

const int COLOR_ENABLE = 48;
const int COLOR_READ = 49;
const int S0 = 50;
const int S1 = 51;
const int S2 = 52;
const int S3 = 53;

const int BASE = 2;
const int SHOULDER_RIGHT = 3;
const int SHOULDER_LEFT = 4;
const int ELBOW = 5;
const int WRIST = 6;
const int GRIPPER_ROTATE = 7;
const int GRIPPER = 8;

const float SHOULDER_LENGTH = 11;
const float ELBOW_LENGTH = 9;
const float WRIST_LENGTH = 14.5;
/* ---------- */

enum WORK_METHODS {
        GIVEN_POSITION,
        COLOR_BASED,
        DIRECT_ANGLE
};

bool started = false;
int workingMethod = GIVEN_POSITION; //method of robot work (1 - sorting based on start/end position, 2 - color sorting, 3 - moving to specified angles by user)

unsigned int colorReading = 0;
int r, g, b;

String serialData;

struct ServoObject {
        Servo servo;
        int minimum;
        int maximum;
        int maxAngle;
        int minAngle;
        float offset;
};

struct ServoObject servos[SERVOS_COUNT];

float previousPercents[SERVOS_COUNT];
float currentPercents[SERVOS_COUNT];
float nextPercents[SERVOS_COUNT];

float receivedAngles[SERVOS_COUNT - 1];
float previousPosition[SERVOS_COUNT];
float nextPosition[SERVOS_COUNT];

int angles[SERVOS_COUNT];

double th_1, th_2, th_3;
float shoulder_angle, elbow_angle;

int moveNumber = 1;
int objectNumber = 0;

float startingPosition[SERVOS_COUNT] = { 0.5, 0.2, 0.2, 0.95, 0.5, 0.5, 0.7 };

vector<float> moves[SERVOS_COUNT];

float redContainer[3];
float greenContainer[3];
float blueContainer[3];

float moveToRedContainer[SERVOS_COUNT];
float moveToGreenContainer[SERVOS_COUNT];
float moveToBlueContainer[SERVOS_COUNT];

vector<float> objectsX;
vector<float> objectsY;
vector<float> objectsZ;
vector<float> objectsDestinationX;
vector<float> objectsDestinationY;
vector<float> objectsDestinationZ;

void initServos() {
        servos[0].servo.attach(BASE);    //Base
        servos[0].minimum = 740;
        servos[0].maximum = 2200;
        servos[0].maxAngle = 180;
        servos[0].minAngle = 0;
        servos[0].offset = 0.0;
        servos[1].servo.attach(SHOULDER_RIGHT);    //Shoulder right
        servos[1].minimum = 800;
        servos[1].maximum = 2200;
        servos[1].minAngle = 25;
        servos[1].maxAngle = 165;
        servos[1].offset = 0.0;
        servos[2].servo.attach(SHOULDER_LEFT);    //Shoulder left
        servos[2].minimum = 825;
        servos[2].maximum = 2140;
        servos[2].minAngle = 25;
        servos[2].maxAngle = 165;
        servos[2].offset = 0.0;
        servos[3].servo.attach(ELBOW);    //Elbow
        servos[3].minimum = 800;
        servos[3].maximum = 2200;
        servos[3].minAngle = 75;
        servos[3].maxAngle = 225;
        servos[3].offset = 0.0;
        servos[4].servo.attach(WRIST);    //Wrist
        servos[4].minimum = 800;
        servos[4].maximum = 2200;
        servos[4].minAngle = 0;
        servos[4].maxAngle = 180;
        servos[4].offset = 0.0;
        servos[5].servo.attach(GRIPPER_ROTATE);    //Wrist rotate
        servos[5].minimum = 1000;
        servos[5].maximum = 2200;
        servos[5].minAngle = 0;
        servos[5].maxAngle = 180;
        servos[5].offset = 0.0;
        servos[6].servo.attach(GRIPPER);    //Gripper
        servos[6].minimum = 1000;
        servos[6].maximum = 2000;
        servos[6].offset = 0.0;
        servos[6].minAngle = 0;
        servos[6].maxAngle = 180;

        int angle = 0;
        for(int i = 0; i < SERVOS_COUNT; i++) {
                if(i == 1) { //in case of right soulder servo the direction is opposite
                        angle = servos[i].minimum + (1.0 - startingPosition[i]) * (servos[i].maximum - servos[i].minimum);
                }
                else {
                        angle = servos[i].minimum + startingPosition[i] * (servos[i].maximum - servos[i].minimum);
                }

                servos[i].servo.writeMicroseconds(angle);
        }
}

void calculateOffsets() {
        for(int i= 0; i < SERVOS_COUNT; i++) {
                servos[i].offset = (nextPercents[i] - previousPercents[i]) / 20.0;
        }
}

void calculateAngles() {
        for(int i = 0; i < SERVOS_COUNT; i++) {
                if(i == 1) { //in case of right soulder servo the direction is opposite
                        angles[i] = servos[i].minimum + (1.0 - currentPercents[i]) * (servos[i].maximum - servos[i].minimum);
                }
                else {
                        angles[i] = servos[i].minimum + currentPercents[i] * (servos[i].maximum - servos[i].minimum);
                }
        }
}

void writeAngles() {
        for(int i = 0; i < SERVOS_COUNT; i++) {
                servos[i].servo.writeMicroseconds(angles[i]);
        }
}

void updatePercents() {
        bool nextMove = true;

        for(int i = 0; i < SERVOS_COUNT; i++) {
                if(previousPercents[i] < nextPercents[i]) {
                        if(currentPercents[i] <= nextPercents[i]) {
                                currentPercents[i] += servos[i].offset;
                                if(servos[i].offset > 0.001 || servos[i].offset < -0.001) {
                                        nextMove = false;
                                }
                        }
                }
                else if(previousPercents[i] > nextPercents[i]) {
                        if(currentPercents[i] >= nextPercents[i]) {
                                currentPercents[i] += servos[i].offset;
                                if(servos[i].offset > 0.001 || servos[i].offset < -0.001) nextMove = false;
                        }
                }
        }

        if(nextMove && workingMethod == GIVEN_POSITION) {
                if(moveNumber < (moves[0].size() - 1)) {
                        moveNumber++;
                }
                else {
                        started = false;
                        Serial.println("MSGRobot has finished work");
                }
                for(int i = 0; i < SERVOS_COUNT; i++) {
                        previousPercents[i] = currentPercents[i];
                        nextPercents[i] = moves[i][moveNumber];
                }
                delay(1000);
        }
}

void putStartingPosition() {
        for(int i = 0; i < SERVOS_COUNT; i++) {
                moves[i].push_back(startingPosition[i]);
        }
}

void calculatePercents() {
        float baseRotation;
        putStartingPosition();
        if(workingMethod == GIVEN_POSITION) {
                for(int i = 0; i < objectsX.size(); i++) {

                        calculateShoulderElbowRotation(i, false);
                        baseRotation = calculateBaseRotation(i, false);

                        moves[0].push_back(baseRotation);
                        moves[1].push_back(startingPosition[1]);
                        moves[2].push_back(startingPosition[2]);
                        moves[3].push_back(startingPosition[3]);
                        moves[4].push_back(startingPosition[4]);
                        moves[5].push_back(startingPosition[5]);
                        moves[6].push_back(startingPosition[6]);

                        moves[0].push_back(baseRotation);
                        moves[1].push_back(shoulder_angle);
                        moves[2].push_back(shoulder_angle);
                        moves[3].push_back(elbow_angle);
                        moves[4].push_back(calculateWristRotation());
                        moves[5].push_back(startingPosition[5]);
                        moves[6].push_back(startingPosition[6]);

                        moves[0].push_back(baseRotation);
                        moves[1].push_back(shoulder_angle);
                        moves[2].push_back(shoulder_angle);
                        moves[3].push_back(elbow_angle);
                        moves[4].push_back(calculateWristRotation());
                        moves[5].push_back(startingPosition[5]);
                        moves[6].push_back(0.2);

                        moves[0].push_back(baseRotation);
                        moves[1].push_back(startingPosition[1]);
                        moves[2].push_back(startingPosition[2]);
                        moves[3].push_back(startingPosition[3]);
                        moves[4].push_back(startingPosition[4]);
                        moves[5].push_back(startingPosition[5]);
                        moves[6].push_back(0.2);

                        calculateShoulderElbowRotation(i, true);
                        baseRotation = calculateBaseRotation(i, true);

                        moves[0].push_back(baseRotation);
                        moves[1].push_back(startingPosition[1]);
                        moves[2].push_back(startingPosition[2]);
                        moves[3].push_back(startingPosition[3]);
                        moves[4].push_back(startingPosition[4]);
                        moves[5].push_back(startingPosition[5]);
                        moves[6].push_back(0.2);

                        moves[0].push_back(baseRotation);
                        moves[1].push_back(shoulder_angle);
                        moves[2].push_back(shoulder_angle);
                        moves[3].push_back(elbow_angle);
                        moves[4].push_back(calculateWristRotation());
                        moves[5].push_back(startingPosition[5]);
                        moves[6].push_back(0.2);

                        moves[0].push_back(baseRotation);
                        moves[1].push_back(shoulder_angle);
                        moves[2].push_back(shoulder_angle);
                        moves[3].push_back(elbow_angle);
                        moves[4].push_back(calculateWristRotation());
                        moves[5].push_back(startingPosition[5]);
                        moves[6].push_back(startingPosition[6]);

                        moves[0].push_back(baseRotation);
                        moves[1].push_back(startingPosition[1]);
                        moves[2].push_back(startingPosition[2]);
                        moves[3].push_back(startingPosition[3]);
                        moves[4].push_back(startingPosition[4]);
                        moves[5].push_back(startingPosition[5]);
                        moves[6].push_back(startingPosition[6]);

                        putStartingPosition();
                }
        }

        int movesLength = moves[0].size();
        objectsX.clear();
        objectsY.clear();
        objectsZ.clear();
        objectsDestinationX.clear();
        objectsDestinationY.clear();
        objectsDestinationZ.clear();


}

float calculateBaseRotation(int index, bool destination) {
        if(objectsX.size() > 0) {
                float x = objectsX[index];
                float y = objectsY[index];
                if(destination == true) {
                        x = objectsDestinationX[index];
                        y = objectsDestinationY[index];
                }

                float angle = atan(x/y) * 180/M_PI;

                return ((90 + angle) / servos[0].maxAngle);
        }

        return 0;
}

float calculateWristRotation() {
        float angle = 0;
        float th_3 = 0;
        th_3 = abs((th_1) - (-th_2-90));
        angle = (180 - th_3 - servos[4].minAngle) / (servos[4].maxAngle - servos[4].minAngle);
        return angle;
}

void calculateShoulderElbowRotation(int index, bool destination) {
        double x, y, z, reach, k_1, k_2, gamma;

        x = objectsX[index];
        y = objectsY[index];
        z = objectsZ[index];

        if(destination == true) {
                x = objectsDestinationX[index];
                y = objectsDestinationY[index];
                z = objectsDestinationZ[index];
        }

        reach = sqrt(x*x + y*y) - WRIST_LENGTH;

        double tmp = (pow(reach, 2) + pow(z, 2) - pow(SHOULDER_LENGTH, 2) - pow(ELBOW_LENGTH, 2))/(2*SHOULDER_LENGTH*ELBOW_LENGTH);
        double tmp2 = -sqrt(1 - pow(tmp, 2));
        th_2 = atan2(tmp2, tmp);
        k_1 = SHOULDER_LENGTH + ELBOW_LENGTH * cos(th_2);
        k_2 = ELBOW_LENGTH * sin(th_2);
        gamma = atan2(k_2, k_1);
        th_1 = abs(atan2(z, reach) - gamma);

        th_1 = th_1 * 180.0/M_PI;
        th_2 = th_2 * 180.0/M_PI;

        shoulder_angle = (180 - th_1- servos[1].minAngle) / (servos[1].maxAngle - servos[1].minAngle);
        elbow_angle = (90-th_2 - servos[3].minAngle) / (servos[3].maxAngle - servos[3].minAngle);
}

/* COLOR SENSOR */
void initColorSensor() {
        pinMode(S0, OUTPUT);
        pinMode(S1, OUTPUT);
        pinMode(S2, OUTPUT);
        pinMode(S3, OUTPUT);
        pinMode(COLOR_READ, INPUT);
        pinMode(COLOR_ENABLE, OUTPUT);

        digitalWrite(COLOR_ENABLE, LOW);
        digitalWrite(S0, HIGH);
        digitalWrite(S1, HIGH);
        digitalWrite(S2, HIGH);
        digitalWrite(S3, LOW);
}

void readColor() {
        // Setting red filtered photodiodes to be read
        digitalWrite(S2,LOW);
        digitalWrite(S3,LOW);
        // Reading the output colorReading
        colorReading = pulseIn(COLOR_READ, LOW);
        //Remaping the value of the colorReading to the RGB Model of 0 to 255
        colorReading = map(colorReading, 5,60,255,0);
        if(colorReading > 255) colorReading = 255;
        r = colorReading;
        // Printing the value on the serial monitor
        delay(100);
        // Setting Green filtered photodiodes to be read
        digitalWrite(S2,HIGH);
        digitalWrite(S3,HIGH);
        // Reading the output colorReading
        colorReading = pulseIn(COLOR_READ, LOW);
        //Remaping the value of the colorReading to the RGB Model of 0 to 255
        colorReading = map(colorReading, 5,80,255,0);
        if(colorReading > 255) colorReading = 255;
        g = colorReading;
        // Printing the value on the serial monitor
        delay(100);
        // Setting Blue filtered photodiodes to be read
        digitalWrite(S2,LOW);
        digitalWrite(S3,HIGH);
        // Reading the output colorReading
        colorReading = pulseIn(COLOR_READ, LOW);
        //Remaping the value of the colorReading to the RGB Model of 0 to 255

        colorReading = map(colorReading, 5,55,255,0);
        if(colorReading > 255) colorReading = 255;
        b = colorReading;
}

void readData() {
        int inData[255];
        bool finished = false;
        int index = 0;
        int reading;

        while(Serial.available() > 0) {
                reading = Serial.read();
                if(reading == 255) index = 0;
                if(reading == 254) finished = true;
                inData[index] = reading;
                index++;

                if(finished) {
                        if(inData[1] == 0) reset();      //reset device
                        else if(inData[1] == 1) start(); //start device
                        else if(inData[1] == 2) setWorkingMethod(COLOR_BASED); //set sorting method to color based
                        else if(inData[1] == 3) setWorkingMethod(GIVEN_POSITION); //disable color based sorting
                        else if(inData[1] == 4) addObject(inData); //add object source and destination to vector
                        else if(inData[1] == 5) setContainers(inData); //set containers position
                        else if(inData[1] == 6) setWorkingMethod(DIRECT_ANGLE);
                        else if(inData[1] == 7) setAngles(inData);
                        finished = false;
                }
        }
}

void sendData() {
        //sending color info
        if(workingMethod == COLOR_BASED) {
                serialData = String("COL" + String(r) + "," + String(g) + "," + String(b));
        }
        //sending angles info
        serialData = "ANG";
        serialData += String(currentPercents[0] * (servos[0].maxAngle - servos[0].minAngle) + servos[0].minAngle) + ",";
        serialData += String(currentPercents[1] * (servos[1].maxAngle - servos[1].minAngle) + servos[1].minAngle) + ",";
        serialData += String(currentPercents[2] * (servos[2].maxAngle - servos[2].minAngle) + servos[2].minAngle) + ",";
        serialData += String(currentPercents[3] * (servos[3].maxAngle - servos[3].minAngle) + servos[3].minAngle) + ",";
        serialData += String(currentPercents[4] * (servos[4].maxAngle - servos[4].minAngle) + servos[4].minAngle) + ",";
        serialData += String(currentPercents[5] * (servos[5].maxAngle - servos[5].minAngle) + servos[5].minAngle) + ",";
        serialData += String(currentPercents[6] * (servos[6].maxAngle - servos[6].minAngle) + servos[6].minAngle);
        Serial.println(serialData);
}

void reset() {
        started = false;
        moveNumber = 0;
        objectsX.clear();
        objectsY.clear();
        objectsZ.clear();
        objectsDestinationX.clear();
        objectsDestinationY.clear();
        objectsDestinationZ.clear();
        for(int i = 0; i < SERVOS_COUNT; i++) moves[i].clear();
        Serial.println("MSGConfiguration cleared");
}

void start() {
        started = true;
        Serial.println("MSGRobot has been started");
        calculatePercents();
        if(workingMethod == GIVEN_POSITION) {
                for(int i = 0; i < SERVOS_COUNT; i++) {
                        previousPercents[i] = moves[i][0];
                        currentPercents[i] = previousPercents[i];
                        nextPercents[i] = moves[i][1];
                }
        }
        else if(workingMethod == DIRECT_ANGLE) {
                for(int i = 0; i < SERVOS_COUNT; i++) {
                        previousPercents[i] = previousPosition[i];
                        currentPercents[i] = previousPercents[i];
                        nextPercents[i] = nextPosition[i];
                }
        }
}

void addObject(int data[255]) {
        String x, y, z, dx, dy, dz;
        objectsX.push_back(data[2] - 128);
        objectsY.push_back(data[3] - 128);
        objectsZ.push_back(data[4] - 128);
        objectsDestinationX.push_back(data[5] - 128);
        objectsDestinationY.push_back(data[6] - 128);
        objectsDestinationZ.push_back(data[7] - 128);
        x = String(objectsX.back());
        y = String(objectsY.back());
        z = String(objectsZ.back());
        dx = String(objectsDestinationX.back());
        dy = String(objectsDestinationY.back());
        dz = String(objectsDestinationZ.back());
        Serial.println("MSGRegistered object (" + x + ", " + y + ", " + z +", " + dx + ", " + dy + ", " + dz +")");
}

void setAngles(int data[255]) {
        if(data[2] < servos[0].minAngle || data[2] > servos[0].maxAngle) {
          Serial.println("MSGBase angle out of range!");
        }
        else receivedAngles[0] = (float)(data[2] - servos[0].minAngle) / (servos[0].maxAngle - servos[0].minAngle);
        
        if(data[3] < servos[1].minAngle || data[3] > servos[1].maxAngle) {
          Serial.println("MSGShoulder angle out of range!");
        }
        else receivedAngles[1] = (float)(data[3] - servos[1].minAngle) / (servos[1].maxAngle - servos[1].minAngle);
        
        if(data[4] < servos[3].minAngle || data[4] > servos[3].maxAngle) {
          Serial.println("MSGElbow angle out of range!");
        }
        else receivedAngles[2] = (float)(data[4] - servos[3].minAngle) / (servos[3].maxAngle - servos[3].minAngle);
        
        if(data[5] < servos[4].minAngle || data[5] > servos[4].maxAngle) {
          Serial.println("MSGWrist angle out of range!");
        }
        else receivedAngles[3] = (float)(data[5] - servos[4].minAngle) / (servos[4].maxAngle - servos[4].minAngle);
        
        if(data[6] < servos[5].minAngle || data[6] > servos[5].maxAngle) {
          Serial.println("MSGGripper angle out of range!");
        }
        else receivedAngles[4] = (float)(data[6] - servos[5].minAngle) / (servos[5].maxAngle - servos[5].minAngle);
        
         if(data[7] < servos[6].minAngle || data[7] > servos[6].maxAngle) {
          Serial.println("MSGGripper angle out of range!");
        }
        else receivedAngles[5] = (float)(data[7] - servos[6].minAngle) / (servos[6].maxAngle - servos[6].minAngle);
 
        previousPosition[0] = nextPosition[0];
        previousPosition[1] = nextPosition[1];
        previousPosition[2] = nextPosition[2];
        previousPosition[3] = nextPosition[3];
        previousPosition[4] = nextPosition[4];
        previousPosition[5] = nextPosition[5];
        previousPosition[6] = nextPosition[6];
        nextPosition[0] = receivedAngles[0];
        nextPosition[1] = receivedAngles[1];
        nextPosition[2] = receivedAngles[1];
        nextPosition[3] = receivedAngles[2];
        nextPosition[4] = receivedAngles[3];
        nextPosition[5] = receivedAngles[4];
        nextPosition[6] = receivedAngles[5];

        for(int i = 0; i < SERVOS_COUNT; i++) {
                previousPercents[i] = previousPosition[i];
                currentPercents[i] = previousPercents[i];
                nextPercents[i] = nextPosition[i];
        }

        calculateOffsets();
}


void setWorkingMethod(int _workingMethod) {
        workingMethod = _workingMethod;

        switch(workingMethod) {
        case GIVEN_POSITION: Serial.println("MSGSorting using start/end position enabled"); break;
        case COLOR_BASED: Serial.println("MSGColor sorting enabled"); break;
        case DIRECT_ANGLE:
                previousPosition[0] = nextPosition[0] = startingPosition[0];
                previousPosition[1] = nextPosition[1] = startingPosition[1];
                previousPosition[2] = nextPosition[2] = startingPosition[2];
                previousPosition[3] = nextPosition[3] = startingPosition[3];
                previousPosition[4] = nextPosition[4] = startingPosition[4];
                previousPosition[5] = nextPosition[5] = startingPosition[5];
                previousPosition[6] = nextPosition[6] = startingPosition[6];
                Serial.println("MSGDirect angle mode enabled"); break;

        default: Serial.println("MSGWrong method!"); break;
        }
}

void setContainers(int data[255]) {
}

void setup()
{
        initServos();
        initColorSensor();

        Serial.begin(9600);
}

void loop()
{
        readData();
        if(started) {
                if(workingMethod == COLOR_BASED) readColor();

                if(workingMethod == GIVEN_POSITION) calculateOffsets();
                calculateAngles();
                writeAngles();

                updatePercents();

                sendData();
        }
        delay(35);
}

