#include "ManipPlanner.hpp"

#include <math.h>

namespace
{
    // the distance for which to consider 
    // a given obsticle
    double s_epsillon = 5;
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
        repulsive(iter, force);

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

    Point closest = {0, 0};
    for (std::int32_t obIter = 0; obIter < m_manipSimulator->GetNrObstacles(); ++obIter)
    {
        closest = m_manipSimulator->ClosestPointOnObstacle(obIter, r_jX, r_jY);
        if (inRange(closest))
        {
            // if the point is close enough to the obsticle at i
            // then consider it's repulsive force in our sum
            p_force.first += r_jX - closest.m_x;
            p_force.second += r_jY - closest.m_y;
        }
    }
}

bool ManipPlanner::inRange(const Point& p_closest) const
{
    double distance = sqrt((p_closest.m_x * p_closest.m_x) + 
                           (p_closest.m_y * p_closest.m_y));
    return distance < s_epsillon ? true : false;
}

void ManipPlanner::attractive(std::int32_t p_index, std::pair< double, double >& p_force) const
{
    p_force.first += m_manipSimulator->GetGoalCenterX() - 
        m_manipSimulator->GetLinkEndX(p_index);
    p_force.second += m_manipSimulator->GetGoalCenterY() - 
        m_manipSimulator->GetLinkEndY(p_index);
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

    for (std::int32_t iter = 0; iter < m_manipSimulator->GetNrLinks(); ++iter)
    {
        startX = m_manipSimulator->GetLinkStartX(iter);
        startY = m_manipSimulator->GetLinkStartY(iter);

        // break this out into pieces so it is easy to understand

        // calculate the 
        x = controlX - startX;
        y = controlY - startY;

        p_jacobian.push_back(std::make_pair(y, (-1 * x)));
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
}

