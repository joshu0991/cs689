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
    m_distToLeavePoint.first = m_distToLeavePoint.second = 0;
    m_distLeaveToGoal = HUGE_VAL;    
    m_dirOfWallFollowing = 1;    
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
    if (m_mode == STRAIGHT)
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
            setDistToGoal();
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
    if (m_mode == STRAIGHT)
    {
        // should we enter wall following
        if (timeToTurn(sensor))
        {
            // we will start wall following
            // mark the hit point
            m_hit[0] = m_simulator->GetRobotCenterX();
            m_hit[1] = m_simulator->GetRobotCenterY();
            
            // to signify wall following for the first round
            m_mode = AROUND_AND_AWAY_FROM_HIT_POINT;

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
    // we are in the first round of wall following ( discovering the closest
    // leave point )
    // we are moving away from the hit point until we encounter the first
    // candidate leave point. then we change m_mode to AROUND
    // this is because we need to find out when we finish our first round
    else if (m_mode == AROUND_AND_AWAY_FROM_HIT_POINT)
    {

        // we want to check if the current point is a candidate for the
        // shortest distance leaving point
        if (!goalObstructed(sensor))
        {
            // calculate the length of this point to the goal
            double distance = calculateDistanceToGoal(sensor);
            if (distance < m_dclosest)
            {
                // mark the new found closest point
                m_dclosest = distance;
                m_closest.first  = m_simulator->GetRobotCenterX();
                m_closest.second = m_simulator->GetRobotCenterY();

                // done wall following return to normal
                m_mode = AROUND;
                m_distToLeavePoint.first = m_distanceTracker;
                m_distToLeavePoint.second = 0;  
            }
        }

        // still need to wall follow this will calculate
        // the next move will be perpendicular to the closest
        // point on the obsticle since the sensor values update
        perpendicularToHit(sensor, headingVector);
        m_distanceTracker++;
        m_distToLeavePoint.second++;  
    }
    // we obtain the closest point and check if we encounter the hit 
    // point again. when we do we change m_mode to show that we are in 
    // the second round
    else if (m_mode == AROUND)
    {

        // circle around the object
        perpendicularToHit(sensor, headingVector);
 
        // calculate the length of this point to the goal
        double distance = calculateDistanceToGoal(sensor);
        if (distance < m_dclosest)
        {
            // mark the new found closest point
            m_dclosest = distance;
            m_closest.first  = m_simulator->GetRobotCenterX();
            m_closest.second = m_simulator->GetRobotCenterY();
            m_distToLeavePoint.first = m_distanceTracker;
            m_distToLeavePoint.second = 0;  
        }
        else if (m_simulator->ArePointsNear(m_simulator->GetRobotCenterX(),
                                            m_simulator->GetRobotCenterY(),
                                            m_hit[0],
                                            m_hit[1]))
        {
            // this means we have one full revolution around the obsticle
            m_mode = AROUND_AND_TOWARD_LEAVE_POINT;
            if (m_distToLeavePoint.first > m_distToLeavePoint.second)
            {
                headingVector.first = -1 * headingVector.first;
                headingVector.second = -1 * headingVector.second;
                m_distToLeavePoint.first = 0;
                m_distToLeavePoint.second = m_distanceTracker = -1;
                m_dirOfWallFollowing = -1;
            }
            else
            {
                m_dirOfWallFollowing = 1;
            }
        }
        m_distanceTracker++;
        m_distToLeavePoint.second++;  
    }
    // as soon as we encounter the previously identified closest leave point
    // we change the mode to straight so we move towards goal. 
    else if (m_mode == AROUND_AND_TOWARD_LEAVE_POINT)
    {
        if (m_simulator->ArePointsNear(m_simulator->GetRobotCenterX(),
                                            m_simulator->GetRobotCenterY(),
                                            m_closest.first,
                                            m_closest.second))
        {
            // we found the closest point so leave and go to goal
            m_mode = STRAIGHT;
            setDistToGoal();
            m_leave[0] = m_simulator->GetRobotCenterX();
            m_leave[1] = m_simulator->GetRobotCenterY();
            
            // reset the distances for the new leavepoint
            m_distToLeavePoint.first = m_distToLeavePoint.second = 0;

            calculateHeadingToGoal(headingVector);
        }
        else
        {
            // circle around the object
            perpendicularToHit(sensor, headingVector);
            headingVector.first = m_dirOfWallFollowing * headingVector.first;
            headingVector.second = m_dirOfWallFollowing * headingVector.second;
            
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
            if (closer(m_simulator->GetRobotCenterX(),
                       m_simulator->GetRobotCenterY(),
                       m_hit[0],
                       m_hit[1]))
            {
                setDistToGoal();
                m_mode = STRAIGHT;
            }
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

    // Calculates the perpandicular vector which identifies the direction of movement
    std::pair< double, double > movingVector;
    perpendicularToHit(p_sensor, movingVector);
    
    // Calculates the vector from the current position to the goal
    std::pair< double, double > headingVector;
    calculateHeadingToGoal(headingVector);
    
    double dotProduct = ( vectorX * headingVector.first ) + ( vectorY * headingVector.second );
    double dotProduct_2 = ( movingVector.first * headingVector.first ) + ( movingVector.second * headingVector.second );
    return ((dotProduct <= 0) && (dotProduct_2 >= 0)) ? false : true;
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

bool BugAlgorithms::closer(double p_x1, double p_y1, double p_x2, double p_y2) const
{
    // get the goal points
    double goalX = m_simulator->GetGoalCenterX();
    double goalY = m_simulator->GetGoalCenterY();
   
    // create the first vector
    double vectorX1 = goalX - p_x1;
    double vectorY1 = goalY - p_y1;
    double magnitude1 = std::sqrt((vectorX1 * vectorX1) +(vectorY1 * vectorY1));
    
    // create the second vector
    double vectorX2 = goalX - p_x2; 
    double vectorY2 = goalY - p_y2; 
    double magnitude2 = std::sqrt((vectorX2 * vectorX2) +(vectorY2 * vectorY2));

    return magnitude1 < magnitude2;
}

void BugAlgorithms::setDistToGoal()
{
    // get robot start points
    double robotStartX = m_simulator->GetRobotInitX();
    double robotStartY = m_simulator->GetRobotInitY();

    // get the goal points
    double goalX = m_simulator->GetGoalCenterX();
    double goalY = m_simulator->GetGoalCenterY();

    // vector
    double vX1 = goalX = robotStartX;
    double vY1 = goalY = robotStartY;

    m_distLeaveToGoal = std::sqrt((vX1 * vX1) + (vY1 * vY1));
}

