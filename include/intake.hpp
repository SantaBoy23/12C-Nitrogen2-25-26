#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

// Declare motors, but do NOT construct them here.
inline pros::Motor intakeBottom(9);
inline pros::Motor intakeMid(-8);
inline pros::Motor intakeTop(21);

inline ez::Piston hood('A');
inline ez::Piston bottom('G');

void IntakeLiftDrop(bool IntakeLiftState);
void HoodLift(bool HoodState);
void BottomContract(bool BottomState);

void BottomIntakeMove(int speed);
void MidIntakeMove(int speed);
void TopIntakeMove(int speed);
void FullIntakeMove(int speed);
void IntakeControl();