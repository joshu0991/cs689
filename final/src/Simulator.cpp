//
//  Simulator.cpp
//  Graphics
//
//  Created by nusha mehmanesh on 4/6/18.
//  Copyright © 2018 nusha mehmanesh. All rights reserved.
//

#include "Simulator.hpp"

Simulator::Simulator(void) :
    triangle(Triangle::getInstance()),
    goal_vertices(3)
{
    goalCenter.first  = 16;
    goalCenter.second = -12;
    
}

Simulator::~Simulator(void)
{
 //nothing for now
}
