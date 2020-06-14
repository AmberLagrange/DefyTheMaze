#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"
#include "QTRSensors.h"
#include "NewPing.h"

#define RANDOM false                //If the robot tries to solve it randomly.

//Robot
#define ROBOT_SIZE  3.5             //Length of robot from middle wheel to tip of ultrasonic sensor.

//Grid
#define WIDTH 4                     //Width of grid in number of intersections.
#define HEIGHT 4                    //Height of grid in number of intersections.
#define GRID_SIZE 8                 //Distance between intersections in inches.
#define NODE_SIZE WIDTH * HEIGHT    //Number of nodes, always WIDTH * HEIGHT.

//IR
#define NUM_SENSORS 2               //Number of infrared sensors the robot has.
#define NUM_SAMPLES_PER_SENSOR 4    //Number of samples each sensor takes.
#define LEFT_IR_PIN 0               //Left infrared pin number.
#define RIGHT_IR_PIN 1              //Right infrared pin number.
#define LEFT_IR 0                   //Left infrared id. Default 0.
#define RIGHT_IR 1                  //Right infrared id. Default 1.

//Motors
#define LEFT_DIR 0                  //Left motor's direction (0 = Forwards, 1 = Backwards). Default 0.
#define LEFT_PWMPIN 6               //Left motor's pwmpin. Default 6.
#define LEFT_DIRPIN 7               //Left motor's dirpin. Default 7.

#define RIGHT_DIR 1                 //Right motor's direction (0 = Forwards, 1 = Backwards). Default 1.
#define RIGHT_PWMPIN 5              //Right motor's pwmpin. Default 5.
#define RIGHT_DIRPIN 4              //Right motor's dirpin. Default 4.

//Ultrasonic
#define ECHO_PIN 2                  //Ultrasonic sensor's echo pin.
#define TRIGGER_PIN 3               //Ultrasonic sensor's trigger pin.
#define MAX_CM_DISTANCE 200         //Ultrasonic sensor's max distance that it can scan in cm. Default 200.

//Directions
#define F 0                         //Forwards.
#define D 1                         //Down.
#define B 2                         //Backwards.
#define U 3                         //Up.

#define INF 10000                   //"Infinity".

class Robot
{
  public:
    static Robot* getInstance(int w, int h);
    static Robot* getInstance();
    static void destroyInstance();

    bool run();
  private:
    static Robot * mInstance;
    Robot(int w, int h);
    Robot(const Robot&);
    Robot& operator=(const Robot&);

    int mPosX;
    int mPosY;
    int mWallX;
    int mWallY;
    int mDirection;
    bool hasRotated;

    int mNodes[NODE_SIZE][NODE_SIZE];
    int mDistances[NODE_SIZE];

    QTRSensorsAnalog mIR;
    unsigned int mSensorValues[NUM_SENSORS];

    Motor mLeftMotor;
    Motor mRightMotor;

    NewPing mUltra;

    void chooseDirection();
    void changeDirection(char newDirection);

    void scan();
    void calcDistance();
    int shortestDistance(bool shortestSet[]);

    void forwards();
    void followLine();
    void rotate(bool cw);

    void randomMovement();
  
    void robotForward()    {mLeftMotor.forward(false), mRightMotor.forward(false);}
    void robotLeft()       {mLeftMotor.reverse(true),  mRightMotor.forward(true);}
    void robotRight()      {mLeftMotor.forward(true),  mRightMotor.reverse(true);}
    void robotSoftLeft()   {mLeftMotor.halt(),         mRightMotor.forward(false);}
    void robotSoftRight()  {mLeftMotor.forward(false), mRightMotor.halt();}
    void robotHalt()       {mLeftMotor.halt(),         mRightMotor.halt();}

    bool isBlack(int val) {return (val >= 300);}
    bool isWhite(int val) {return (val < 300);}
};

#endif //ROBOT_H
