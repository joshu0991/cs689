/**
 *@file BugAlgorithms.hpp
 *@brief Prototype for the Bug algorithms required in this assignment
 */

#ifndef BUG_ALGORITHMS_HPP_
#define BUG_ALGORITHMS_HPP_

#include "Simulator.hpp"

#include <cstdint>
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

    /*!
     * Given a sensor reading calculate the distance the point is from the goal
     * \param[in] p_sensor the sensor reading to calculate
     * \return the distance from the sensor point to the goal
     */
    double calculateDistanceToGoal(Sensor p_sensor) const;

    /*!
     *  Given a vector represented by a pair make it a unit vector
     *  \param[in, out] p_headingVector the vector to make a unit vector
     *                  the unit vector is contained in the same pair
     */
     void makeUnitVector(std::pair< double, double >& p_headingVector) const;

    /*!
     * Check if a point is close to the line created from the robot start point
     * to the goal
     * \param[in] p_x the x coordinate to check
     * \param[in] p_y the y coordinate to check
     * \return true iff the point given by p_x, p_y is close to the line created
     *         by the robot start point and goal point
     */
    bool checkPointOnLine(double p_x, double p_y) const;

protected:
    /**
     *@brief Pointer to simulator
     */
    Simulator  *m_simulator;
    
    /*!
     * \brief tracks the closest point from the obsticle to the goal
     */
    std::pair<double, double> m_closest;
    
    //! the closest distance we have seen from the obsticle to the goal
    double m_dclosest;

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
    
    //! counter to track how many moves we are away from the hit point
    std::uint16_t m_distanceTracker;

    //! true if we have seen the hit point after a revolution
    bool m_haveHitPoint;

    friend class Graphics;
};

#endif
