#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

// Declare motors, but do NOT construct them here.
inline pros::Motor intakeBottom(1);
inline pros::Motor intakeMid(-4);
inline pros::Motor intakeTop(3);

inline ez::Piston center('B');
inline ez::Piston bottom('G');

void IntakeLiftDrop(bool IntakeLiftState);
void CenterDrop(bool CenterState);
void BottomContract(bool BottomState);

void BottomIntakeMove(int speed);
void MidIntakeMove(int speed);
void TopIntakeMove(int speed);
void FullIntakeMove(int speed);
void IntakeControl();