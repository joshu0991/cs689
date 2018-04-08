#ifndef MANIP_PLANNER_HPP_
#define MANIP_PLANNER_HPP_

#include "ManipSimulator.hpp"

#include <cstdint> // std int
#include <utility> // std pair

class ManipPlanner
{
public:
    ManipPlanner(ManipSimulator * const manipSimulator);
            
    ~ManipPlanner(void);

/*
 * This is the function that you should implement.
 * This function needs to compute by how much the link angles should change
 * so that the robot makes a small move toward the goal while avoiding
 * obstacles, as guided by the potential field.
 *
 * allLinksDeltaTheta(j) should contain the small delta change for the angle
 * associated with the j-th link.
 *
 * Note that the attractive potential should be defined only between the end
 * effector point on the manipulator and the goal center. 
 *
 * The repulsive potential on the other hand should be defined between each
 * obstacle and each link end.
 *
 * This will ensure that, when possible, the end effector moves toward the
 * goal while every link avoids collisions with obstacles.
 *
 * You have access to the simulator.
 * You can use the methods available in simulator to get all the information
 * you need to correctly implement this function
 *
 */
    void ConfigurationMove(double allLinksDeltaTheta[]);
    
private:
    /*!
     * Calcualte the rep. force
     *
     * \param[in] p_index the index of the link to calculate for
     * \param[out] p_force the returned summed force
     */
    void repulsive(std::int32_t p_index, std::pair< double, double >& p_force) const;

    /*!
     * Check if a distane is in range and should be considered
     * \param[in] p_dist the distance to check
     * \return true iff dist < epsillon
     */
    bool inRange(double p_dist) const;

    /*!
     * Calculate the attactive force for an index
     * \param[in] p_index the index to check
     * \param[out] p_force the return summed forces
     */
    void attractive(std::int32_t p_index, std::pair< double, double >& p_force) const; 

    /*!
     * Set up the jacobian tranpose
     * \param[in] p_controlIndex the control index to to use in the jacobian
     * \param[out] p_jacobian the returned jacobian matrix
     */
    void setupJacobian(std::int32_t p_controlIndex, 
                       std::vector< std::pair< double, double > >& p_jacobian) const;

    /*!
     * Multiply the jacobian transpose by a force vector
     * \param[in] p_force the force component to multiply
     * \param[in] p_jacobianT the transposed jacobian matrix
     * \param[out] p_sumUqDeltas the returned sum of the delta values
     */
    void forceJacobianMult(const std::pair< double, double >& p_force, 
                           const std::vector< std::pair< double, double > >& p_jacobianT,
                           double p_sumUqDeltas[]) const;

    /*!
     * Normalize a a vector
     * \param[in, out] p_retSums the vectr to normalize
     */
    void normalizeVector(double p_retSums[]) const;

    /*!
     * Normalize a 2d force vector
     * \param[in] p_force the force vector to normalize
     */
    void normalizeForce(std::pair< double, double >& force) const;

protected:    
    
    ManipSimulator  *m_manipSimulator;
};

#endif
