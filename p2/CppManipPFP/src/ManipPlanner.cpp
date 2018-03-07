#include "ManipPlanner.hpp"

#include <math.h>
#include <iostream>

namespace
{
    // the distance for which to consider 
    // a given obsticle
    double s_epsillon = 3;

    double s_scalFactor_att = 0.005;
    double s_scalFactor_rep = 0.9;
}

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;   
}

ManipPlanner::~ManipPlanner(void)
{
    //do not delete m_simulator  
}

void ManipPlanner::ConfigurationMove(double allLinksDeltaTheta[])
{
    // accumulator for force
    std::pair< double, double > force;
    
    // for modeling the jacobian
    std::vector< std::pair< double, double > > jacobianT;
    
    // for each link
    for (std::int32_t iter = 0; iter < m_manipSimulator->GetNrLinks(); ++iter)
    {
        if (iter == m_manipSimulator->GetNrLinks() - 1)
        {
            // this link has the end effector hence we
            // need attactive and repulsive forces
            attractive(iter, force);
        }
        std::cout << iter << "attractive force x" <<force.first << "\n";
        std::cout << iter <<"attractive force y" <<force.second << "\n";
        repulsive(iter, force);
        std::cout << "rep force x" <<force.first << "\n";
        std::cout << "rep force y" <<force.second << "\n";

        // calculate the u_j for this link and keep 
        // the running sum in allLinksDeltaTheta
        setupJacobian(iter, jacobianT);

        // multiply the force by the jacobian and store the result
        // in our sum vector which is our dTheta
        forceJacobianMult(force, jacobianT, allLinksDeltaTheta);

        // at the end of this link clear the force point
        force.first = 0;
        force.second = 0;
    }
}

void ManipPlanner::repulsive(std::int32_t p_index, std::pair< double, double >& p_force) const
{
    // get the x and y coordinates for the control point that we are yet
    // to consider on this link (the end point of the link)
    double r_jX = m_manipSimulator->GetLinkEndX(p_index);
    double r_jY = m_manipSimulator->GetLinkEndY(p_index);
    std::pair< double, double > rep_force;

    std::cout << "I'M CALCULATING REPULSIVE\n";
    Point closest = {0, 0};
    for (std::int32_t obIter = 0; obIter < m_manipSimulator->GetNrObstacles(); ++obIter)
    {
        std::cout << "I'M inloop\n";
        closest = m_manipSimulator->ClosestPointOnObstacle(obIter, r_jX, r_jY);
        std::cout << "closest x " << closest.m_x << "closest y " << closest.m_y << "\n";
        std::cout << "rj x " << r_jX << "rj y " << r_jY << "\n";
        if (inRange(closest, r_jX, r_jY))
        {
        std::cout << "I'M inrange\n";
            // if the point is close enough to the obsticle at i
            // then consider it's repulsive force in our sum
            rep_force.first += (r_jX - closest.m_x);
            rep_force.second += (r_jY - closest.m_y);
        }
    }
    //this normalizes the Sum of Forces for the current point
    normalizeForce(rep_force);
    p_force.first += s_scalFactor_rep * rep_force.first;
    p_force.second += s_scalFactor_rep * rep_force.second;

}

bool ManipPlanner::inRange(const Point& p_closest, double x, double y) const
{
    double distance = sqrt(((x - p_closest.m_x) *  (x - p_closest.m_x)) +
                           ((y - p_closest.m_y) *  (y - p_closest.m_y)));
    return distance < s_epsillon ? true : false;
}

void ManipPlanner::attractive(std::int32_t p_index, std::pair< double, double >& p_force) const
{
    p_force.first += ((m_manipSimulator->GetGoalCenterX() - 
        m_manipSimulator->GetLinkEndX(p_index)));
    p_force.second += ( (m_manipSimulator->GetGoalCenterY() - 
        m_manipSimulator->GetLinkEndY(p_index)));
    normalizeForce(p_force);
    p_force.first *= s_scalFactor_att;
    p_force.second *= s_scalFactor_att;
}

void ManipPlanner::setupJacobian(std::int32_t p_indexControl, 
                                 std::vector< std::pair< double, double > >& p_jacobian) const
{
    // ensure the jacobian vector is empty
    p_jacobian.clear();

    double startX = 0;
    double startY = 0;
    double x = 0;
    double y = 0;
    double controlX = m_manipSimulator->GetLinkEndX(p_indexControl);
    double controlY = m_manipSimulator->GetLinkEndY(p_indexControl);

    for (std::int32_t iter = 0; iter <= p_indexControl; ++iter)
    {
        startX = m_manipSimulator->GetLinkStartX(iter);
        startY = m_manipSimulator->GetLinkStartY(iter);

        // break this out into pieces so it is easy to understand

        // calculate the 
        x = controlX - startX;
        y = controlY - startY;

        p_jacobian.push_back(std::make_pair((-1 * y), x));
    }

    for (std::int32_t iter = p_indexControl+1; iter < m_manipSimulator->GetNrLinks(); ++iter)
    {
        p_jacobian.push_back(std::make_pair(0, 0));
    }
}

void ManipPlanner::forceJacobianMult(const std::pair< double, double >& p_force,
                                     const std::vector< std::pair< double, double > >& p_jacobianT,
                                     double p_sumUqDeltas[]) const
{
    std::uint32_t counter = 0;
    for (std::vector< std::pair< double, double > >::const_iterator jacobianIterator = p_jacobianT.begin();
         jacobianIterator < p_jacobianT.end(); ++jacobianIterator)
    {
        // the element at this index is the x,y pair to multiply by the force to
        // obtain our n by matrix sum which is stored in sumUqDeltas
        p_sumUqDeltas[counter++] += ((p_force.first * jacobianIterator->first) + 
                                     (p_force.second * jacobianIterator->second));
    }
    normalizeVector(p_sumUqDeltas);
}

void ManipPlanner::normalizeVector(double p_retSums[]) const
{
    double delta = 0.005;
    
    for (std::int32_t iter = 0; iter < m_manipSimulator->GetNrLinks(); ++iter)
    {
        if (p_retSums[iter] > 0)
        {
            p_retSums[iter] = delta;
        }
        else
        {
            p_retSums[iter] = -1 * delta;
        }
    }
}

void ManipPlanner::normalizeForce(std::pair< double, double >& force) const
{
    double magnitude = 0;
    
    magnitude = (force.first * force.first) + (force.second * force.second);
    magnitude = sqrt(magnitude);

    if (magnitude > 0)
    {
        force.first /= magnitude;
        force.second /= magnitude;
    }
}
