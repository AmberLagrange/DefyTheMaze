#include "Robot.h"

Robot *Robot::mInstance = 0;

Robot* Robot::getInstance(int w, int h)
{
  if (!mInstance) mInstance = new Robot(w, h);
  return mInstance;
}

Robot* Robot::getInstance()
{
  if (!mInstance) mInstance = new Robot(0, 0);
  return mInstance;
}

void Robot::destroyInstance()
{
  if (mInstance) mInstance = 0;
}

//Constructor
Robot::Robot(int w, int h) :
  mPosX(0), mPosY(0), mWallX(w - 1), mWallY(h - 1), mDirection(F),
  mLeftMotor(Motor(LEFT_DIR, LEFT_PWMPIN, LEFT_DIRPIN)), mRightMotor(Motor(RIGHT_DIR, RIGHT_PWMPIN, RIGHT_DIRPIN)),
  mIR((unsigned char*)(const unsigned char[]) {LEFT_IR_PIN, RIGHT_IR_PIN}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR),
  mUltra(TRIGGER_PIN, ECHO_PIN, MAX_CM_DISTANCE)

{
  for (int i = 0; i < NODE_SIZE; i++)
  {
    for (int j = 0; j < NODE_SIZE; j++)
      mNodes[i][j] = 0;

    if (i - WIDTH >= 0) mNodes[i][i - WIDTH] = 1;
    if (i + WIDTH <= NODE_SIZE - 1) mNodes[i][i + WIDTH] = 1;
    if (i % WIDTH != 0) mNodes[i][i - 1] = 1;
    if (i % WIDTH != WIDTH - 1) mNodes[i][i + 1] = 1;
  }

  calcDistance();
}

//Random movement
void Robot::randomMovement()
{
  int s1 = random(0, 200);
  int s2 = random(0, 200);

  int d1 = random(0, 2);
  int d2 = random(0, 2);

  switch(d1)
  {
    case 0:
      mLeftMotor.forward(0);
      break;
    case 1:
      mLeftMotor.halt();
      break;
    case 2:
      mLeftMotor.reverse(0);
      break;
  }

  switch(d2)
  {
    case 0:
      mRightMotor.forward(0);
      break;
    case 1:
      mRightMotor.halt();
      break;
    case 2:
      mRightMotor.reverse(0);
      break;
  }

  mLeftMotor.setRotSpeed(s1);
  mRightMotor.setRotSpeed(s2);
}

//Choose the direction for the robot, rotate to said direction, then move in that direction
bool Robot::run()
{
  if(RANDOM)
  {
    randomMovement();
    return false;
  }
  
  if (mPosX == mWallX && mPosY == mWallY)
  {
    destroyInstance();
    return true;
  }

  hasRotated = false;

  do chooseDirection();
  while (hasRotated);

  mPosX += (mDirection == F) - (mDirection == B);
  mPosY += (mDirection == D) - (mDirection == U);

  forwards();
  return false;
}

//Decide which direction the robot should face
void Robot::chooseDirection()
{
  scan();

  int currentNode = (mPosY * WIDTH) + mPosX;
  int dist = INF;
  int nodes[2];
  int dir[2];

  nodes[1] = -INF;

  for (int i = 0; i < NODE_SIZE; i++)
  {
    if (mNodes[currentNode][i] && mDistances[i] < dist)
    {
      dist = mDistances[i];
      nodes[0] = i;
      nodes[1] = -INF;
    }
    else if(mNodes[currentNode][i] && mDistances[i] == dist) nodes[1] = i;
  }

  for (int i = 0; i < 2; i++)
  {
    if (nodes[i] == -INF) break;

    if (nodes[i] - 1 == currentNode) dir[i] = F;
    else if (nodes[i] - WIDTH == currentNode) dir[i] = D;
    else if (nodes[i] + 1 == currentNode) dir[i] = B;
    else if (nodes[i] + WIDTH == currentNode) dir[i] = U;
  }

  if (nodes[1] != -INF)
  {
    int turn[2];

    for (int i = 0; i < 2; i++)
    {
      if (mDirection == dir[i]) turn[i] = 0;
      else if (abs(mDirection - dir[i]) != 2) turn[i] = 1;
      else turn[i] = 2;
    }

    if (turn[0] < turn[1]) changeDirection(dir[0]);
    else changeDirection(dir[1]);
  }
  else changeDirection(dir[0]);
}

//Rotate the robot how many times if needed, then note if the robot was rotated
void Robot::changeDirection(char newDirection)
{
  if (mDirection == newDirection)
  {
    hasRotated = false;
    return;
  }
  rotate(newDirection > mDirection != (abs(newDirection - mDirection) == 3));
  if (abs(newDirection - mDirection) == 2) rotate(newDirection > mDirection);
  mDirection = newDirection;
  hasRotated = true;
}

//Scans for obstacles with the ultrasonic sensor, then updates the nodes if they are broken from other nodes
void Robot::scan()
{
  double distance = (double)round(2 * (mUltra.ping_in() + ROBOT_SIZE) / ((double)(GRID_SIZE))) / 2;
  if (distance == 0) distance = NODE_SIZE * GRID_SIZE + 1;

  if (mDirection == F && distance + mPosX <= mWallX) //Break an intersection between 2 nodes
  {
    mNodes[WIDTH * mPosY + mPosX + (int)(distance - 0.5)][WIDTH * mPosY + mPosX + (int)(distance + 0.5)] = 0;
    mNodes[WIDTH * mPosY + mPosX + (int)(distance + 0.5)][WIDTH * mPosY + mPosX + (int)(distance - 0.5)] = 0;
  }
  else if (mDirection == D && distance + mPosY <= mWallY) //Break an intersection between 2 nodes
  {
    mNodes[mPosX + (mPosY + (int)(distance - 0.5)) * WIDTH][mPosX + (mPosY + (int)(distance + 0.5)) * WIDTH] = 0;
    mNodes[mPosX + (mPosY + (int)(distance + 0.5)) * WIDTH][mPosX + (mPosY + (int)(distance - 0.5)) * WIDTH] = 0;
  }
  else return;

  calcDistance();
}

//Calculate the distances for all the nodes in the maze
void Robot::calcDistance() //Dijkstra's algorithm
{
  bool shortestPathSet[NODE_SIZE];  //List of bools that say if a given node's shortest distance has been found

  for (int i = 0; i < NODE_SIZE; i++) //Set all nodes' distance to infinity and say that their shortest path haven't been found
  {
    mDistances[i] = INF;
    shortestPathSet[i] = false;
  }

  mDistances[NODE_SIZE - 1] = 0;  //Set the distance to the end to 0

  for (int i = 0; i < NODE_SIZE - 1; i++)
  {
    int s = shortestDistance(shortestPathSet); //Find a node that has the shortest distance to the source
    shortestPathSet[s] = true; //Say that you dound that node's minimal distance

    for (int n = 0; n < NODE_SIZE; n++) //For all the adjacent nodes, update their new distances using the new found node
      if (!shortestPathSet[n] && mNodes[s][n] && mDistances[s] != INF && mDistances[s] + mNodes[s][n] < mDistances[n]) mDistances[n] = mDistances[s] + mNodes[s][n];
  }
}

//Calculate the shortest distance to the end for a node
int Robot::shortestDistance(bool shortestPathSet[])
{
  int dist = INF;
  int node;

  for (int n = 0; n < NODE_SIZE; n++)
    if (!shortestPathSet[n] && mDistances[n] < dist)
    {
      dist = mDistances[n];
      node = n;
    }

  return node;
}

//Follow the line, then once the robot reaches the next intersection, keep going for 180ms
void Robot::forwards()
{
  robotForward();

  mIR.read(mSensorValues);

  while (!(isBlack(mSensorValues[LEFT_IR]) && isBlack(mSensorValues[RIGHT_IR]))) followLine();

  int startTime = millis();
  int currentTime = millis();

  while (currentTime - startTime < 180)
  {
    followLine();
    currentTime = millis();
  }

  robotHalt();
}

void Robot::followLine()
{
  mIR.read(mSensorValues);

  if (isWhite(mSensorValues[LEFT_IR]) && isWhite(mSensorValues[RIGHT_IR])) robotForward();
  else if (isBlack(mSensorValues[LEFT_IR]) && isWhite(mSensorValues[RIGHT_IR])) robotSoftLeft();
  else if (isWhite(mSensorValues[LEFT_IR]) && isBlack(mSensorValues[RIGHT_IR])) robotSoftRight();
}

void Robot::rotate(bool cw)
{
  int sensor;

  if (cw) sensor = RIGHT_IR, robotRight();
  else sensor = LEFT_IR, robotLeft();

  while (isBlack(mSensorValues[sensor])) mIR.read(mSensorValues);
  while (isWhite(mSensorValues[sensor])) mIR.read(mSensorValues);
  while (isBlack(mSensorValues[sensor])) mIR.read(mSensorValues);
  robotHalt();
}

