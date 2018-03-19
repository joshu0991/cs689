#include "ManipPlanner.hpp"

#include <math.h>
#include <iostream>

namespace
{
    // the distance for which to consider 
    // a given obsticle
    double s_epsillon = 0.8;

    double s_scalFactor_att = 0.0001;
    double s_scalFactor_rep = 0.005;
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
    for(size_t i=0;i<m_manipSimulator->GetNrLinks();++i)
     {
         allLinksDeltaTheta[i]=0;
     }

    
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
        std::cout << "f sum repulsive X" << iter << " = " << force.first << "\n";
                    std::cout << "f sum repulsive Y" << iter << " = " << force.second << "\n";

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
    normalizeVector(allLinksDeltaTheta);
}

void ManipPlanner::repulsive(std::int32_t p_index, std::pair< double, double >& p_force) const
{
    // get the x and y coordinates for the control point that we are yet
    // to consider on this link (the end point of the link)
    double r_jX = m_manipSimulator->GetLinkEndX(p_index);
    double r_jY = m_manipSimulator->GetLinkEndY(p_index);
    //std::pair< double, double > rep_force;
    std::pair< double, double > temp;

    Point closest = {0, 0};
    for (std::int32_t obIter = 0; obIter < m_manipSimulator->GetNrObstacles(); ++obIter)
    {
        closest = m_manipSimulator->ClosestPointOnObstacle(obIter, r_jX, r_jY);
        
        //calculating the distance from the obstacle to the point
        double distance = sqrt(((closest.m_x - r_jX) *  (closest.m_x - r_jX)) +
                           ((closest.m_y - r_jY) *  (closest.m_y - r_jY)));
        if (inRange(distance))
        {
            // if the point is close enough to the obsticle at i
            // then consider it's repulsive force in our sum
            temp.first = (closest.m_x - r_jX);
            temp.second = (closest.m_y - r_jY);
            normalizeForce(temp);
            p_force.first += s_scalFactor_rep * temp.first;
            p_force.second += s_scalFactor_rep * temp.second;
        }
    }
}

bool ManipPlanner::inRange( double distance ) const
{
    return distance < s_epsillon ? true : false;
}

void ManipPlanner::attractive(std::int32_t p_index, std::pair< double, double >& p_force) const
{
    p_force.first += ((m_manipSimulator->GetLinkEndX(p_index) - 
        m_manipSimulator->GetGoalCenterX() ));
    p_force.second += ((m_manipSimulator->GetLinkEndY(p_index) -
       m_manipSimulator->GetGoalCenterY()));
    //normalizeForce(p_force);
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
    std::cout << " x " << iter << " = " << (-1*y) << "\n";
            std::cout << " y " << iter << " = " << x << "\n";
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
        std::cout << "Ux " << counter << " = " << p_force.first << "\n";
        std::cout << "Uy " << counter << " = " << p_force.second << "\n";
         std::cout << " jx " << counter << " = " << jacobianIterator->first << "\n";
         std::cout << " jy " << counter << " = " << jacobianIterator->second << "\n";
         std::cout << " ***  = " << p_sumUqDeltas[counter] << "\n";


        p_sumUqDeltas[counter++] += ((p_force.first * jacobianIterator->first) + 
                                     (p_force.second * jacobianIterator->second));
        std::cout << " ***  = " << p_sumUqDeltas[counter-1] << "\n";
    }
}

void ManipPlanner::normalizeVector(double p_retSums[]) const
{
    //double delta = 0.005;
    
    
    for (std::int32_t iter = 0; iter < m_manipSimulator->GetNrLinks(); ++iter)
    {
       /* if (p_retSums[iter] > 0)
        {
            p_retSums[iter] = delta;
        }
        else
        {
            p_retSums[iter] = -1 * delta;
        }*/
        std::cout << " sum,  " << iter << " = " << p_retSums[iter] << "\n";
        p_retSums[iter] *= -1;
    }
}

void ManipPlanner::normalizeForce(std::pair< double, double >& force) const
{
    double distance = 0;
    
    distance = (force.first * force.first) + (force.second * force.second);
    //distance = sqrt(distance);

    if (distance > 0)
    {
        force.first /= distance;
        force.second /= distance;
    }
}
