#include "main.h"

void setIntake(int intakePower){

    intake11W.move(intakePower);
}

void setBanda(int bandaPower){

    frontmotor.move(bandaPower);
}
