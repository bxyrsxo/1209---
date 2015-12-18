#pragma once

#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include <iostream>
#include <math.h>
#include "System.h"


//  Add Function prototypes Here

void Gravity_Compensator( int ARM, float Gcomp[6] );
void Forward_Kinematic( int ARM, float end_effector_pos[3] );



#endif