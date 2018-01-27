#include "BugAlgorithms.hpp"

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
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
            calculateHeadingToGoal(headingVector);
        }
    }

    // calculate the move. \TODO We should probably unit vector these
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
       // if (goalObstructed(sensor))
        {
            // still need to wall follow this will calculate
            // the next move will be perpendicular to the closest
            // point on the obsticle since the sensor values update
            perpendicularToHit(sensor, headingVector);
        }
   /*     else
        {
            // done wall following return to normal
            m_mode = STRAIGHT;
            calculateHeadingToGoal(headingVector);
        }*/
    }

    move.m_dx = step * headingVector.first;
    move.m_dy = step * headingVector.second;

    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{

    //add your implementation
    Move move ={0,0};
    
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
}
        
