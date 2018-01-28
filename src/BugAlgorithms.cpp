#include "BugAlgorithms.hpp"

#include <complex>
#include <iostream>

BugAlgorithms::BugAlgorithms(Simulator * const simulator) :
    m_closest(),
    m_dclosest(HUGE_VAL),
    m_distanceTracker(false)
{
    m_simulator = simulator;   
    //add your initialization of other variables
    //that you might have declared
    
    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;    
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator  
}

Move BugAlgorithms::Bug0(Sensor sensor)
{
    // cache for conveniance
    Move move = {0, 0};
    std::pair< double, double > headingVector;
    double step = m_simulator->GetStep();
   
    // wall following checks 
    if (m_mode != AROUND)
    {
        // should we enter wall following
        if (timeToTurn(sensor))
        {
            // we will start wall following
            // mark the hit point
            m_hit[0] = sensor.m_xmin;
            m_hit[1] = sensor.m_ymin;
            
            // to signify wall following
            m_mode = AROUND;

            // calculate a perpendicular vector
            perpendicularToHit(sensor, headingVector);
        }
        else
        {
            // this is the normal case with no wall following
            // calculate the heading vector
            calculateHeadingToGoal(headingVector);
        }
    }
    else if (m_mode == AROUND)
    {
        // we must check to see if we can go to the goal if not
        // we continue wall following
        if (goalObstructed(sensor))
        {
            // still need to wall follow this will calculate
            // the next move will be perpendicular to the closest
            // point on the obsticle since the sensor values update
            perpendicularToHit(sensor, headingVector);
        }
        else
        {
            // done wall following return to normal
            m_mode = STRAIGHT;
            m_leave[0] = m_simulator->GetRobotCenterX();
            m_leave[1] = m_simulator->GetRobotCenterY();
            calculateHeadingToGoal(headingVector);
        }
    }

    makeUnitVector(headingVector);

    // calculate the move. 
    move.m_dx = step * headingVector.first;
    move.m_dy = step * headingVector.second;

    return move;
}

Move BugAlgorithms::Bug1(Sensor sensor)
{
    // cache for conveniance
    Move move = {0, 0};
    std::pair< double, double > headingVector;
    double step = m_simulator->GetStep();
   
    // wall following checks 
    if (m_mode != AROUND)
    {
        m_distanceTracker = 0;
        m_haveHitPoint = false;
        // should we enter wall following
        if (timeToTurn(sensor))
        {
            // we will start wall following
            // mark the hit point
            m_hit[0] = sensor.m_xmin;
            m_hit[1] = sensor.m_ymin;
            
            // to signify wall following
            m_mode = AROUND;

            // calculate a perpendicular vector
            perpendicularToHit(sensor, headingVector);
        }
        else
        {
            // this is the normal case with no wall following
            // calculate the heading vector
            calculateHeadingToGoal(headingVector);
        }
    }
    else if (m_mode == AROUND)
    {
        // circle around the object
        perpendicularToHit(sensor, headingVector);
        ++m_distanceTracker;
 
        // calculate the length of this point to the goal
        double distance = calculateDistanceToGoal(sensor);
        if (distance < m_dclosest)
        {
            // mark the new found closest point
            m_dclosest = distance;
            m_closest.first  = m_simulator->GetRobotCenterX();
            m_closest.second = m_simulator->GetRobotCenterY();
        }
        else if (m_distanceTracker > 20 &&
                 m_simulator->ArePointsNear(sensor.m_xmin,
                                            sensor.m_ymin,
                                            m_hit[0],
                                            m_hit[1]))
        {
            // this means we have one full revolution around the obsticle
            m_haveHitPoint = true;
            m_distanceTracker = 0;
        }
        else if (m_simulator->ArePointsNear(m_simulator->GetRobotCenterX(),
                                            m_simulator->GetRobotCenterY(),
                                            m_closest.first,
                                            m_closest.second) &&
                 m_haveHitPoint)
        {
            // we found the closest point so leave and go to goal
            m_mode = STRAIGHT;
            m_leave[0] = m_simulator->GetRobotCenterX();
            m_leave[1] = m_simulator->GetRobotCenterY();
        }
    }

    makeUnitVector(headingVector);

    // calculate the move. 
    move.m_dx = step * headingVector.first;
    move.m_dy = step * headingVector.second;
    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{
    // cache for conveniance
    Move move = {0, 0};
    std::pair< double, double > headingVector;
    double step = m_simulator->GetStep();
   
    // wall following checks 
    if (m_mode != AROUND)
    {
        m_distanceTracker = 0;
        m_haveHitPoint = false;
        // should we enter wall following
        if (timeToTurn(sensor))
        {
            // we will start wall following
            // mark the hit point
            m_hit[0] = sensor.m_xmin;
            m_hit[1] = sensor.m_ymin;
            
            // to signify wall following
            m_mode = AROUND;

            // calculate a perpendicular vector
            perpendicularToHit(sensor, headingVector);
        }
        else
        {
            // this is the normal case with no wall following
            // calculate the heading vector
            calculateHeadingToGoal(headingVector);
        }
    }
    else if (m_mode == AROUND)
    {
        // wall follow until we find the line
        perpendicularToHit(sensor, headingVector);

        // check to see if we are back to the line
        if (checkPointOnLine(m_simulator->GetRobotCenterX(), 
                             m_simulator->GetRobotCenterY()))
        {
            m_mode = STRAIGHT;
        }
    }

    makeUnitVector(headingVector);

    // calculate the move. 
    move.m_dx = step * headingVector.first;
    move.m_dy = step * headingVector.second;
    return move;
}

void BugAlgorithms::calculateHeadingToGoal(std::pair< double, double >& p_headingVector) const
{
    // get the robots coordinates
    double robotX = m_simulator->GetRobotCenterX();
    double robotY = m_simulator->GetRobotCenterY();

    // get the goal coordinates
    double goalX = m_simulator->GetGoalCenterX();
    double goalY = m_simulator->GetGoalCenterY();

    // calculate the vector
    p_headingVector.first  = goalX - robotX;
    p_headingVector.second = goalY - robotY;
}

bool BugAlgorithms::timeToTurn(Sensor p_sensor) const
{
    return m_simulator->GetWhenToTurn() <= p_sensor.m_dmin ? false : true;
}

void BugAlgorithms::perpendicularToHit(Sensor p_sensor, std::pair< double, double >& p_headingVector) const
{
    // get the robots coordinates
    double robotX = m_simulator->GetRobotCenterX();
    double robotY = m_simulator->GetRobotCenterY();
  
    // get the obsticle points closest to us
    double obX = p_sensor.m_xmin;
    double obY = p_sensor.m_ymin;

    // calculate the vector that collides with this point
    double vectorX = obX - robotX;
    double vectorY = obY - robotY;

    // take the vector orthagonal to the vector that crashes
    p_headingVector.first  = vectorY;
    p_headingVector.second = -1 * vectorX; 
}

bool BugAlgorithms::goalObstructed(Sensor p_sensor) const
{
    // Calculates the vector from the current position to the obsticle

    // get the robots coordinates
    double robotX = m_simulator->GetRobotCenterX();
    double robotY = m_simulator->GetRobotCenterY();
  
    // get the obsticle points closest to us
    double obX = p_sensor.m_xmin;
    double obY = p_sensor.m_ymin;

    // calculate the vector that collides with this point
    double vectorX = obX - robotX;
    double vectorY = obY - robotY;

    // Calculates the vector from the current position to the goal

    std::pair< double, double > headingVector;
    calculateHeadingToGoal(headingVector);
    
    double dotProduct = ( vectorX * headingVector.first ) + ( vectorY * headingVector.second );
    double vectorMagnitude = std::sqrt( ( vectorX * vectorX ) + ( vectorY * vectorY ) );
    double vectorProjection = dotProduct / vectorMagnitude;
    return vectorProjection < 0 ? false : true;
}

double BugAlgorithms::calculateDistanceToGoal(Sensor p_sensor) const
{
    // get the obsticle points closest to us
    double obX = p_sensor.m_xmin;
    double obY = p_sensor.m_ymin;

    // get the goal coordinates
    double goalX = m_simulator->GetGoalCenterX();
    double goalY = m_simulator->GetGoalCenterY();

    // calculate the vector
    double vectorX  = obX - goalX;
    double vectorY  = obY - goalY;

    // find the magnitude
    return std::sqrt((vectorX * vectorX) + (vectorY * vectorY));
}

void BugAlgorithms::makeUnitVector(std::pair< double, double >& p_headingVector) const
{
    double vectorLength = std::sqrt((p_headingVector.first * p_headingVector.first) + 
                                    (p_headingVector.second * p_headingVector.second));
    // calculate the move. 
    p_headingVector.first /= vectorLength;
    p_headingVector.second /= vectorLength;
}

bool BugAlgorithms::checkPointOnLine(double p_x, double p_y) const
{
    // get robot start points
    double robotStartX = m_simulator->GetRobotInitX();
    double robotStartY = m_simulator->GetRobotInitY();

    // get the goal points
    double goalX = m_simulator->GetGoalCenterX();
    double goalY = m_simulator->GetGoalCenterY();

    //check if our point is close to these points
    return m_simulator->IsPointNearLine(p_x, p_y, robotStartX, robotStartY, goalX, goalY); 
}
