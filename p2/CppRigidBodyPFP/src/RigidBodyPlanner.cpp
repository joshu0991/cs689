#include "RigidBodyPlanner.hpp"

#include <cstdint>
#include <math.h>

namespace
{
    // the step size or speed of the robot for translation
    double s_stepTranslate = 0.006;

    // the angular speed of the robot for rotation
    double s_stepRotate = 0.0006;

    // the max distance for which we will calculate repulsive
    // force for a given obsticle
    double s_epsillon = 5;
}

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
    m_simulator = simulator;   
}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator  
}


RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
    
{
    std::pair< double, double > forces;
    std::tuple< double, double, double > u_qSum;
    std::tuple< double, double, double > u_qCalc;

    // for each "control point"/vertex calculate the net force 
    // and multiply the vector by the jacobian to put it in the workspace
    const double* verticies = m_simulator->GetRobotVertices();
    for (std::int32_t iter = 0; iter < m_simulator->GetNrRobotVertices(); iter+=2)
    {
        // calculate the attactive force to the goal
        attactive(forces, verticies[iter], verticies[iter + 1]);

        // calculate the repulsive force for all obsticles that are
        // within epsillon of this control point
        repulsive(forces, verticies[iter], verticies[iter + 1]);
        
        // the repulsive and attactive forces have been summed for this control point at this point

        // calculate the jacobian transpose of this force
        jacobianMult(u_qCalc, forces, verticies[iter], verticies[iter + 1], m_simulator->GetRobotTheta());

        // add to the running sum of u_q the u_1 value we just calculated for this control point
        std::get< 0 >(u_qSum) = std::get< 0 >(u_qSum) + std::get< 0 >(u_qCalc);
        std::get< 1 >(u_qSum) = std::get< 1 >(u_qSum) + std::get< 1 >(u_qCalc);
        std::get< 2 >(u_qSum) = std::get< 2 >(u_qSum) + std::get< 2 >(u_qCalc);
    
        // these will be reused for the next calcualtion
        // note we don't need to reset the tuples since they
        // are simply overwritten
        forces.first = 0;
        forces.second = 0;
    } 

    // put the tuple into the correct format
    RigidBodyMove move;

    // calculate the move
    move.m_dx = s_stepTranslate * std::get< 0 >(u_qSum);
    move.m_dy = s_stepTranslate * std::get< 1 >(u_qSum);
    move.m_dtheta = s_stepRotate * std::get< 2 >(u_qSum);

    return move;
}

void RigidBodyPlanner::attactive(std::pair< double, double >& p_potential,
                                 double p_x,
                                 double p_y) const 
{
    // get the goal point 
    double goalX = m_simulator->GetGoalCenterX();
    double goalY = m_simulator->GetGoalCenterY();

    //
    p_potential.first += goalX - p_x;
    p_potential.second += goalY - p_y;
}

void RigidBodyPlanner::repulsive(std::pair< double, double >& p_potential,
                                 double p_x,
                                 double p_y) const 
{
    Point ret = {0, 0};
    for (std::uint32_t iter = 0; iter < m_simulator->GetNrObstacles(); ++iter)
    {
        ret = m_simulator->ClosestPointOnObstacle(iter, p_x, p_y);

        if (inRange(ret))
        {
            p_potential.first += (p_x - ret.m_x);
            p_potential.second += p_y - ret.m_y;
        }
    }

}

void RigidBodyPlanner::jacobianMult(std::tuple< double, double, double >& p_retVector,
                                    const std::pair< double, double >& p_force,
                                    double p_x,
                                    double p_y,
                                    double p_theta) const
{
    // theta = p_force.first * (x_j*sin(theta) - y_j*cos(theta)) + 
    //         p_force.second * (x_j*cos(theta) - y_j*sin(theta))
    double dTheta = (p_force.first * (p_x * sin(p_theta) - p_y * cos(p_theta))) +
                    (p_force.second * (p_x * cos(p_theta) - p_y * sin(p_theta)));

    p_retVector = std::make_tuple(p_force.first, p_force.second, dTheta);
}

bool RigidBodyPlanner::inRange(const Point& p_point) const
{
    // check to see if the point is close enough to be considered

    // find the vector between the robot and the point
    double vectorX = m_simulator->GetRobotX() - p_point.m_x;
    double vectorY = m_simulator->GetRobotY() - p_point.m_y;

    // get the distance
    double distance = sqrt((vectorX * vectorX) + (vectorY * vectorY));

    // check to see if the obsticle is close enough to consider
    return distance < s_epsillon ? true : false;
}

