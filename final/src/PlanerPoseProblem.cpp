//
//  PlanerPoseProblem.cpp
//  Graphics
//
//  Created by nusha mehmanesh on 4/6/18.
//  Copyright © 2018 nusha mehmanesh. All rights reserved.
//

#include "PlanerPoseProblem.hpp"

PlanerPoseProblem::PlanerPoseProblem(Simulator * const simulator)
{
    m_simulator = simulator;
    //add your initialization of other variables
    //that you might have declared

}

PlanerPoseProblem::~PlanerPoseProblem(void)
{
    //do not delete m_simulator
}

Move PlanerPoseProblem::PPPAlgorithm()
{
    //use M_PI for angles
    
    double f_x = m_simulator->GetGoalCenter().first -  m_simulator->GetCurrCenterOfMass().first;
    double f_y = m_simulator->GetGoalCenter().second -  m_simulator->GetCurrCenterOfMass().second;
    
    double magnitude = sqrt(f_x*f_x + f_y*f_y);
    
    f_x /= magnitude;
    f_y /= magnitude;
    
    
    Move move = {f_x, f_y, 0};
    return move;
}
