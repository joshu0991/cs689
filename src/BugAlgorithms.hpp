/**
 *@file BugAlgorithms.hpp
 *@brief Prototype for the Bug algorithms required in this assignment
 */

#ifndef BUG_ALGORITHMS_HPP_
#define BUG_ALGORITHMS_HPP_

#include "Simulator.hpp"

// for std::pair
#include <utility>

/**
 * @brief Bug algorithm computes a small move (m_dx, m_dy) that the robot needs to make
 */
struct Move
{
    double m_dx;
    double m_dy;    
};


/**
 *@brief Prototype for the different Bug algorithms required in this assignment
 *
 *@remark
 *  Feel free to add additional functions/data members as you see fit
 */
class BugAlgorithms
{
public:
    /**
     *@brief Set simulator
     *@param simulator pointer to simulator
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator
     */
    BugAlgorithms(Simulator * const simulator);
            
    /**
     *@brief Free memory and delete objects allocated by this instance
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator.
     */
    ~BugAlgorithms(void);
     
    
    /**
     *@brief Select the appropriate move so that the robot behaves
     *       as described in the respective bug algorithms.
     *@param sensor provides closest point from obstacle boundary to robot center
     */
    Move Bug0(Sensor sensor);
    Move Bug1(Sensor sensor);
    Move Bug2(Sensor sensor);
   
    /*!
     * \brief a function to calculate the direction vector to the goal 
     *        when there does not exist an object near.
     * \param[out] p_headingVector on complettion first (x) and second (y)
     *             will contain a representation of a vector in the direction
     *             of the goal
     */
    void calculateHeadingToGoal(std::pair< double, double >& p_headingVector) const;

    /*!
     * \brief function to determine if the robot is close to an obsticle
     *        indication that it is time to turn
     * \param[in] p_sensor a sensor object with the points closest to the
     *            robot center
     * \return true iff the robot is close to an object
     */
    bool timeToTurn(Sensor p_sensor) const;

    /*!
     * \brief calculates a vector orthagonal to the vector created from
     *        the robots center and an object hit point
     * \param[in] p_sensor the sensor object containing which points are
     *            closest to the robot center
     * \param[out] p_headingVector the return x (first) and y (second) 
     *             values representing a vector orthagonal to the hit point
     *             and robot center
     */
    void perpendicularToHit(Sensor p_sensor, std::pair< double, double >& p_headingVector) const;

    /*!
     * \brief determines if the robot can see the goal from it's current position
     *        or if it is obstructed by an obsticle
     * \param[in] p_sensor an object with the obsticles closest x and y points to the robot
     * \return true iff the the robot can't see the goal 
     */
    bool goalObstructed(Sensor p_sensor) const;

protected:
    /**
     *@brief Pointer to simulator
     */
    Simulator  *m_simulator;

    enum Mode
	{
	    STRAIGHT,
	    STRAIGHT_AND_AWAY_FROM_LEAVE_POINT,
	    AROUND_AND_AWAY_FROM_HIT_POINT,
	    AROUND_AND_TOWARD_LEAVE_POINT,
	    AROUND
	};

    double m_hit[2], m_leave[2], m_distLeaveToGoal;
    int    m_mode;
    

    friend class Graphics;
};

#endif
