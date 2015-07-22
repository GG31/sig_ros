#include "include/matrix.hpp"

matrix::matrix() {
   m_row = 4;
   m_col = 4;
/*   m_values = new float*[m_row];
   for (int i=0; i<row; i++) {
      m_values[i] = new float(m_col);
   }*/
}

void matrix::setRotation(float theta) {
   m_values[0][0] = 1;
   m_values[0][1] = 0;
   m_values[0][2] = 0;
   m_values[0][3] = 0;

   m_values[1][0] = 0;
   m_values[1][1] = cos(theta);
   m_values[1][2] = -sin(theta);
   m_values[1][3] = 0;

   m_values[2][0] = 0;
   m_values[2][1] = sin(theta);
   m_values[2][2] = cos(theta);
   m_values[2][3] = 0;

   m_values[3][0] = 0;
   m_values[3][1] = 0;
   m_values[3][2] = 0;
   m_values[3][3] = 1;
}

void matrix::setTranslation(float x, float y, float z) {
   m_values[0][0] = 1;
   m_values[0][1] = 0;
   m_values[0][2] = 0;
   m_values[0][3] = x;

   m_values[1][0] = 0;
   m_values[1][1] = 1;
   m_values[1][2] = 0;
   m_values[1][3] = y;

   m_values[2][0] = 0;
   m_values[2][1] = 0;
   m_values[2][2] = 1;
   m_values[2][3] = z;

   m_values[3][0] = 0;
   m_values[3][1] = 0;
   m_values[3][2] = 0;
   m_values[3][3] = 1;
}

void matrix::reverseTranslation() {
   for (int i=0; i<3; i++) {
      m_values[i][3] *= -1;
   }
}

std::string matrix::display() {
   std::string s = "";
   for (int i=0; i<4; i++) {
      for (int j=0; j<4; j++) {
         std::ostringstream ss;
         ss << m_values[i][j];
         s += ss.str() + " ";
      }
      s += "\n";
   }
   return s;
}

float matrix::get(int r, int c) {
   return m_values[r][c];
}

void matrix::setValue(int r, int c, float v) {
   m_values[r][c] = v;
}

float matrix::mulVec(float a1, float a2, float a3, float a4, float b1, float b2, float b3, float b4) {
   return a1*b1 + a2*b2 + a3*b3 + a4*b4;
}

matrix* matrix::mul(matrix* a) {
   matrix* sol = new matrix();
   for (int i=0; i<4; i++) {
      for (int j=0; j<4; j++) {
         float v = mulVec(m_values[i][0], m_values[i][1], m_values[i][2], m_values[i][3], a->get(0,j), a->get(1,j), a->get(2,j), a->get(3,j)) ;
         sol->setValue(i,j,v);
      }
   }
   return sol;
}

Vector3d matrix::mul(Vector3d& b) {
   float v[4];
   for (int i=0; i<4; i++) {
      v[i] = mulVec(m_values[i][0], m_values[i][1], m_values[i][2], m_values[i][3], b.x(), b.y(), b.z(), 1);
   }
   return Vector3d(v[0], v[1], v[2]);
}
