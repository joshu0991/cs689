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
    glm::vec2 axis_of_rotation;
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
    
    void SetStepAngle(void);
    void SetEdgeVectors(void);
    void GetMaxReorientAngle(void);
    float AngleBetweenVectors( glm::vec2 v1, glm::vec2 v2 );
    glm::mat2 GetRotationMatrix( float angle, bool CW );
    void CalculateCis(void);
    void CalculatePeshkinDistance(void);
    void SetTotalOrientationChange(void);
    void SetUnitReorientPushes(void);
    void CalculateUnitNormals(void);
    void LPSolutionCW(void);
    void LPSolutionCCW(void);
    void ProduceMovesCW(void);
    void ProduceMovesCCW(void);



    
protected:
    /**
     *@brief Pointer to simulator
     */
    Simulator  *m_simulator;
    
    friend class Graphics;
};

#endif /* PlanerPoseProblem_hpp */
