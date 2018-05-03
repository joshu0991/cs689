//
//  PlanerPoseProblem.cpp
//  Graphics
//
//  Created by nusha mehmanesh on 4/6/18.
//  Copyright © 2018 nusha mehmanesh. All rights reserved.
//

#include "PlanerPoseProblem.hpp"
#include <glm/glm.hpp>

PlanerPoseProblem::PlanerPoseProblem(Simulator * const simulator)
{
    m_simulator = simulator;
    //add your initialization of other variables
    //that you might have declared

}

PlanerPoseProblem::~PlanerPoseProblem(void)
{
    //do not delete m_simulator
}

void PlanerPoseProblem::CalcluateRadiusFunction(void)
{
    // to calculate the local minima of the radius function the magnitude of the
    // center of mass to vertices are calculated along with the perpandicular distance of
    // the center of mass to each edge
    for( int i = 0 ; i < 3 ; ++i )
    {
        m_simulator->local_minima[2*i] = glm::length( m_simulator->init_triangle.vertices[i] - 
                                                   m_simulator->init_triangle.com );
    } 

    glm::vec2 edge_v = glm::vec2( 0.0, 0.0);
    glm::vec2 vertexToCom_v = glm::vec2( 0.0, 0.0);
    glm::vec2 Temp = glm::vec2(0.0,0.0);
    for( int i = 1 ; i < 6 ; i+=2 )
    {
        // calculate the edge vector
        vertexToCom_v = m_simulator->init_triangle.com - m_simulator->init_triangle.vertices[i-1];
        edge_v = m_simulator->init_triangle.vertices[i+1] - m_simulator->init_triangle.vertices[i-1];
        Temp = ( glm::dot( vertexToCom_v, edge_v) / glm::length(edge_v) ) * glm::normalize(edge_v);
        m_simulator->local_minima[i] =  glm::length ( (m_simulator->init_triangle.vertices[i-1] + Temp) - 
                                                      m_simulator->init_triangle.com );

    }

    // calculate the maximum reorient angle in CW and CCW direction for the longest stable edge

    // edge_v = v3 - v2 vertexToCom = com - v2
    edge_v = m_simulator->init_triangle.vertices[2] - m_simulator->init_triangle.vertices[1];
    vertexToCom_v = m_simulator->init_triangle.com - m_simulator->init_triangle.vertices[1];
    // calculate the perpandicular CCW vector to the vertexToCom
    Temp = glm::vec2( -vertexToCom_v[1], vertexToCom_v[0]);
    // set the CCW max reorient angle
    m_simulator->e1_max_angle[1] = glm::acos( glm::dot( edge_v, Temp) / ( glm::length(edge_v)*glm::length(Temp) ) );
    // set the CCW reorientation vector
    m_simulator->unit_reorient_CCW.push_back( Temp );
    m_simulator->mag_reorient_CCW.push_back( 1.0 );
    
    // edge_v = v2 - v3 vertexToCom = com - v3
    edge_v = m_simulator->init_triangle.vertices[1] - m_simulator->init_triangle.vertices[2];
    vertexToCom_v = m_simulator->init_triangle.com - m_simulator->init_triangle.vertices[2];
    // calculate the perpandicular CW vector to the vertexToCom
    Temp = glm::vec2( vertexToCom_v[1], -vertexToCom_v[0]);
    // set the CW max reorient angle
    m_simulator->e1_max_angle[0] = glm::acos( glm::dot( edge_v, Temp) / ( glm::length(edge_v)*glm::length(Temp) ) );
    // set the CW reorientation vector
    m_simulator->unit_reorient_CW.push_back( Temp );
    m_simulator->mag_reorient_CW.push_back( 1.0 );

    printf( "\n CCW = %f , CW= %f \n" , m_simulator->e1_max_angle[1], m_simulator->e1_max_angle[0] );


}

void PlanerPoseProblem::SetTotalOrientationChange( )
{
    // calculating the CW and CCW orientation required to reach the goal orientation
    // e1 = v2 - v3 
    // e1 vector is calculated in the initial and goal pose of the triangle
    glm::vec2 e1_goal = m_simulator->goal_triangle.vertices[1] - m_simulator->goal_triangle.vertices[2];
    glm::vec2 e1_init = m_simulator->init_triangle.vertices[1] - m_simulator->init_triangle.vertices[2];
    // calculating the cross product of the two edges to identify the angle direction
    // e1_init x e1_goal if <0 == CCW else >0 == CW 
    glm::vec3 e1_cross = glm::cross( glm::vec3( e1_init, 0.0) , glm::vec3( e1_goal, 0.0) );
    
    bool e1_CW = false;
    if( e1_cross[2] < 0 )
    {
        //set CW flag
        e1_CW = true;
    }
    else if( e1_cross[2] > 0 )
    {
        //set CCW
        e1_CW = false;
    }

    // calculating the angle between the e1 of the initial and goal pose of the triangle
    float e1_dot = glm::dot( e1_init, e1_goal );
    float e1_theta = glm::acos( e1_dot / ( glm::length(e1_init) * glm::length(e1_goal) ) ); 
    
    // Net change in orientation for CCW and CW rotations
    if(e1_CW == true) 
    {
        m_simulator->delta_angle[0] = e1_theta;
        m_simulator->delta_angle[1] = 2*M_PI -m_simulator->delta_angle[0];
    }
    else
    {
        m_simulator->delta_angle[1] = e1_theta;
        m_simulator->delta_angle[0] = 2*M_PI - m_simulator->delta_angle[1];
    }


}
void PlanerPoseProblem::SetUnitReorientPushes( )
{
    // the reorient push angles should be less that the maximum reorient push angles by a marginal angle
    // knowing the step_angle, the total reorientation required to reach the goal orientation the CW and
    // CCW unit push vectors are computed
    
    float marginal_angle = 10 * M_PI/180;
    m_simulator->step_angle[0] = m_simulator->e1_max_angle[0] - marginal_angle;
    m_simulator->step_angle[1] = m_simulator->e1_max_angle[1] - marginal_angle;

    float num_step_pushes = m_simulator->delta_angle[0] / m_simulator->step_angle[0];
    float last_push_angle = fmod( m_simulator->delta_angle[0] , m_simulator->step_angle[0] );
   
    //calculating the sin and cos of theta
    float ctheta = glm::cos( marginal_angle );
    float stheta = glm::sin( marginal_angle ); 

    glm::mat2 Rotation_CCW = glm::mat2( ctheta, -stheta,  //first column
                                        stheta,  ctheta);  //second column 
    glm::mat2 Rotation_CW = glm::mat2( ctheta, stheta,  //first column
                                      -stheta, ctheta);  //second column 
    
    // initial adjustment to the first unit push
    m_simulator->unit_reorient_CW[0] = glm::normalize( m_simulator->unit_reorient_CW[0] * Rotation_CW );
    m_simulator->unit_reorient_CCW[0] = glm::normalize( m_simulator->unit_reorient_CCW[0] * Rotation_CCW );
    
    // calculating the unit CW reorient pushes 
    ctheta = glm::cos( m_simulator->step_angle[0] );
    stheta = glm::sin( m_simulator->step_angle[0] );
    Rotation_CW = glm::mat2( ctheta, stheta,  //first column
                            -stheta, ctheta);  //second column 
    int i = 0;
    for( i = 1 ; i < num_step_pushes; ++i)
    {
        m_simulator->unit_reorient_CW.push_back( glm::normalize( m_simulator->unit_reorient_CW[i-1] * Rotation_CW ) );
    }

    ctheta = glm::cos( last_push_angle );
    stheta = glm::sin( last_push_angle );
    Rotation_CW = glm::mat2( ctheta, stheta,  //first column
                            -stheta, ctheta);  //second column 

    m_simulator->unit_reorient_CW.push_back( glm::normalize( m_simulator->unit_reorient_CW[i-1] * Rotation_CW ) );
    
    //calculating the unit CCW reorient pushes
    num_step_pushes = m_simulator->delta_angle[1] / m_simulator->step_angle[1];
    last_push_angle = fmod( m_simulator->delta_angle[1], m_simulator->step_angle[1] );
    
    ctheta = glm::cos( m_simulator->step_angle[1] );
    stheta = glm::sin( m_simulator->step_angle[1] );
    Rotation_CCW = glm::mat2( ctheta,-stheta,  //first column
                              stheta, ctheta);  //second column 
    
    for( i = 1 ; i < num_step_pushes; ++i)
    {
        m_simulator->unit_reorient_CCW.push_back( glm::normalize( m_simulator->unit_reorient_CCW[i-1] * Rotation_CCW ) );
    }

    ctheta = glm::cos( last_push_angle );
    stheta = glm::sin( last_push_angle );
    Rotation_CCW = glm::mat2( ctheta,-stheta,  //first column
                              stheta, ctheta);  //second column 

    m_simulator->unit_reorient_CCW.push_back( glm::normalize( m_simulator->unit_reorient_CCW[i-1] * Rotation_CCW ) );


}

Move PlanerPoseProblem::PPPAlgorithm(bool CW_rotation)
{
    //use M_PI for angles
    glm::vec2 temp = m_simulator->GetGoalCenter() - m_simulator->GetCurrCenterOfMass();
    temp = glm::normalize(temp);
    
    
    Move move = {temp[0], temp[1], 0};
    return move;
}
