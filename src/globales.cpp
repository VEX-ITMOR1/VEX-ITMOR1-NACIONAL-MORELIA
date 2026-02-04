#include "main.h"
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

pros::Motor intake11W(10, pros::v5::MotorGears::blue);
pros::Motor stage2(9, pros::v5::MotorGears::green);
pros::Motor bigroller(1, pros::v5::MotorGears::blue);
pros::Motor frontmotor(2, pros::v5::MotorGears::blue);
pros::Optical optico(7);


pros::adi::Pneumatics tapa('G', false);
pros::adi::Pneumatics lift1('E', false);
pros::adi::Pneumatics lift2('H', false);
pros::adi::Pneumatics cargador('F', false);