#include "include/GeneralController.hpp"

extern "C"  GeneralController * createController ()
{
   return new GeneralController;
}
