#ifndef RIGID_BODY_PLANNER_HPP_
#define RIGID_BODY_PLANNER_HPP_

#include "RigidBodySimulator.hpp"

#include <tuple> // std tuple
#include <utility> // std pair

struct RigidBodyMove
{
    double m_dx;
    double m_dy;
    double m_dtheta;
};

class RigidBodyPlanner
{
public:
    RigidBodyPlanner(RigidBodySimulator * const simulator);
            
    ~RigidBodyPlanner(void);

    /*
     * This is the function that you should implement.
     * This function needs to compute by how much the position (dx, dy) 
     * and orientation (dtheta) should change so that the robot makes a small 
     * move toward the goal while avoiding obstacles, 
     * as guided by the potential field.
     *
     * You have access to the simulator.
     * You can use the methods available in simulator to get all the information
     * you need to correctly implement this function
     *
     */
    RigidBodyMove ConfigurationMove(void);

private:
    /*! 
     * Calcualte the f_att which guides the robot to the goal
     * this operates on each control point
     * 
     * \param[out] p_potential the vector to store the attactive potential
     * \param[in] p_x the x coordinate contorl point
     * \param[in] p_y the y coordinate control point
     */
    void attactive(std::pair< double, double >& p_move, double p_x, double p_y) const;

    /*!
     * Calculate the f_rep which repulses the robot from obstivles
     * this operates on each control point
     *
     * \param[out] p_potential the vector to store the attactive potential
     * \param[in] p_x the x coordinate contorl point
     * \param[in] p_y the y coordinate control point
     * 
     */
    void repulsive(std::pair< double, double >& p_move, double p_x, double p_y) const;

    /*!
     * Multiply the values in p_force by the jacobian
     *
     * \param[out] p_retValues the return values after multiplying by the jacobian
     *             the first is dx, second is dy and third is dtheta
     * \param[in] p_force the calculated x and y forces for a control point
     * \param[in] p_x the x value for a control point
     * \param[in] p_y the y value for a control point
     * \param[in] p_theta the theta value for the robot
     *
     */
    void jacobianMult(std::tuple< double, double, double >& p_retValues,
                      const std::pair< double, double >& p_force,
                      double p_x,
                      double p_y,
                      double p_theta) const;

    /*!
     * Determine if the given point on an obsticle is close enough 
     * to consider the obsticle in the calculation of the repulsive force
     *
     * \param[in] p_point the point to calculate the distance for and check
     */
    bool inRange(const Point& p_point) const;

    /*!
     * Create a unit vector from our tuple
     *
     * \param[out] p_retValues a tuple containing x, y and theta
     *
     */
    void unitVector(std::tuple< double, double, double >& p_retValues) const;

    /*!
     * Determine if the robot is stuck in the current configuration
     *
     * \return true iff the robot appears to be stuck
     */
    bool checkStuck();

protected:
    RigidBodySimulator *m_simulator;

    //! The last point to check to see if we are stuck
    std::pair< double, double > m_lastPoint;

    // Counter for determining if we surpass the stuck threshold
    std::uint32_t m_stuckCounter;
};

#endif
