#include "include/SimObjController.hpp"

extern "C"  Controller * createController ()
{
   return new SimObjController;
}
