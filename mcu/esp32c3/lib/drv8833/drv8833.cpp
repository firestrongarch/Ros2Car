#include "drv8833.h"

drv8833::drv8833(const int &A1, const int &A2,const int &eep):A1_{A1},A2_{A2},eep_{eep}
{
  pinMode(A1_,OUTPUT);
  pinMode(A2_,OUTPUT);
  pinMode(eep_,OUTPUT);

  digitalWrite(eep_,HIGH);
  digitalWrite(A1_,LOW);
  digitalWrite(A2_,LOW);
}

drv8833::drv8833(const int &A1, const int &A2, const int &PB1, const int &B2, const int &eep):
A1_{A1},A2_{A2},B1_{PB1},B2_{B2},eep_{eep}
{
  pinMode(A1_,OUTPUT);
  pinMode(A2_,OUTPUT);
  pinMode(B1_,OUTPUT);
  pinMode(B2_,OUTPUT);
  pinMode(eep_,OUTPUT);
  pinMode(12,OUTPUT);

  digitalWrite(eep_,HIGH);
  digitalWrite(A1_,LOW);
  digitalWrite(A2_,LOW);
  digitalWrite(B1_,LOW);
  digitalWrite(B2_,LOW);
}

void drv8833::updateSpeed(int num,int speed)
{
  int IN1,IN2;
  if(num == 1){
    IN1 = A1_;
    IN2 = A2_;
  }
  else if(num == 2){
    IN1 = B1_;
    IN2 = B2_;
  }
  else{
    return;
  }

  if(speed > 0){
    digitalWrite(IN1,speed);
    analogWrite(IN2,0);
  }
  else if(speed < 0){
    digitalWrite(IN1,0);
    analogWrite(IN2,-speed);
  }
  else if(speed == 0){
    digitalWrite(IN1,0);
    digitalWrite(IN2,0);
  }
}

void drv8833::updateState(volatile byte state)
{
  digitalWrite(eep_, state);
  digitalWrite(12,state);
}
