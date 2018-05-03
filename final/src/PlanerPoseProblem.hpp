//
//  PlanerPoseProblem.hpp
//  Graphics
//
//  Created by nusha mehmanesh on 4/6/18.
//  Copyright © 2018 nusha mehmanesh. All rights reserved.
//

#ifndef PlanerPoseProblem_hpp
#define PlanerPoseProblem_hpp

#include <stdio.h>

#include "Simulator.hpp"

#include <cstdint>
// for std::pair
#include <utility>

/**
 * @brief PPP algorithm computes a small move (m_dx, m_dy, m_dtheta) that the polygon needs to make
 */
struct Move
{
    float m_dx;
    float m_dy;
    float m_dtheta;
};


class PlanerPoseProblem
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
    PlanerPoseProblem(Simulator * const simulator);
    
    /**
     *@brief Free memory and delete objects allocated by this instance
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator.
     */
    ~PlanerPoseProblem(void);
    
    void CalcluateRadiusFunction(void);
    void SetTotalOrientationChange(void);
    void SetUnitReorientPushes(void);

    /**
     *@brief Select the appropriate move so that the polygon behaves
     *       as described in the planer pose problem
     *@param CW_rotation the desired rotations should be Clockwise
     */
    Move PPPAlgorithm( bool CW_rotation );


    
protected:
    /**
     *@brief Pointer to simulator
     */
    Simulator  *m_simulator;
    
    friend class Graphics;
};

#endif /* PlanerPoseProblem_hpp */
