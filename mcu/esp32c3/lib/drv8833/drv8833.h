#pragma once
#include <Arduino.h>

class drv8833
{
private:
  int A1_,A2_,B1_,B2_,eep_;
public:
  drv8833(const int &A1,const int &A2,const int &eep);
  drv8833(const int &A1,const int &A2,const int &PB1,const int &B2,const int &eep);
  void updateSpeed(int num,int speed);
  void updateState(volatile byte state);
};
