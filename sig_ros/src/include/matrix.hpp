#ifndef MATRIX_H
#define MATRIX_H
#include <stdio.h>
#include <math.h>
#include <string>
#include <sstream>
#include <iostream>

#include "Controller.h"
class matrix {
   private:
      int m_row;
      int m_col;
      float m_values [4][4];
      float mulVec(float a1, float a2, float a3, float a4, float b1, float b2, float b3, float b4);
   public:
      matrix();
      void setRotation(float);
      void setTranslation(float, float, float);
      std::string display();
      float get(int r, int c); 
      void setValue(int r, int c, float v);
      matrix* mul(matrix* a);
      Vector3d mul(Vector3d& b);
      void reverseTranslation();
};

#endif
