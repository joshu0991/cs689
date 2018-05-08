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

void PlanerPoseProblem::SetEdgeVectors(void)
{
    // 0 -> V2V1 
    m_simulator->edge_vectors.push_back( m_simulator->init_triangle.vertices[1] - m_simulator->init_triangle.vertices[2]);
    // 1 -> V2V0
    m_simulator->edge_vectors.push_back( m_simulator->init_triangle.vertices[0] - m_simulator->init_triangle.vertices[2]);
    // 2 -> V0V1
    m_simulator->edge_vectors.push_back( m_simulator->init_triangle.vertices[1] - m_simulator->init_triangle.vertices[0]);
    
    // 0 -> V2COM
    m_simulator->edge_com_vectors.push_back( m_simulator->init_triangle.com - m_simulator->init_triangle.vertices[2] );
    // 1 -> V1COM
    m_simulator->edge_com_vectors.push_back( m_simulator->init_triangle.com - m_simulator->init_triangle.vertices[1] );
    // 2 -> V0COM
    m_simulator->edge_com_vectors.push_back( m_simulator->init_triangle.com - m_simulator->init_triangle.vertices[0] );
}

void PlanerPoseProblem::GetMaxReorientAngle(void)
{
    // calculate the maximum reorient angle in CW and CCW direction for the longest stable edge
    
    // CW
    // calculate the perpandicular CW vector to V2Com
    glm::vec2 perp = glm::vec2( m_simulator->edge_com_vectors[0][1] , -m_simulator->edge_com_vectors[0][0] );
    m_simulator->max_reorient_angles = glm::vec2(  AngleBetweenVectors( perp , m_simulator->edge_vectors[0] ), 0.0 );

    // CCW 
    // caculate the perpandicular vector to V1Com
    perp = glm::vec2( -m_simulator->edge_com_vectors[1][1] , m_simulator->edge_com_vectors[1][0] );
    m_simulator->max_reorient_angles[1] = AngleBetweenVectors( perp , -m_simulator->edge_vectors[0] );
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

    //printf( "\n CCW = %f , CW= %f \n" , m_simulator->e1_max_angle[1], m_simulator->e1_max_angle[0] );



}

void PlanerPoseProblem::SetTotalOrientationChange( )
{
    // calculating the CW and CCW orientation required to reach the goal orientation
    // e1 = v2 - v3 
    // e1 vector is calculated in the initial and goal pose of the triangle
    glm::vec2 e1_goal = m_simulator->goal_triangle.vertices[1] - m_simulator->goal_triangle.vertices[2];
    glm::vec2 e1_init = m_simulator->edge_vectors[0];
    // calculating the cross product of the two edges to identify the angle direction
    // e1_init x e1_goal if <0 == CCW else >0 == CW 
    // gsls calculates the cross of arg2 X arg1
    glm::vec3 e1_cross = glm::cross( glm::vec3( e1_init, 0.0), glm::vec3( e1_goal, 0.0));
    
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
    float e1_theta = AngleBetweenVectors( e1_init, e1_goal);
    
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

void PlanerPoseProblem::SetStepAngle( )
{
    // the reorient push angles should be less that the maximum reorient push angles by a marginal angle
    float marginal_angle = 10 * M_PI/180;
    m_simulator->step_angle[0] = m_simulator->max_reorient_angles[0] - marginal_angle;
    m_simulator->step_angle[1] = m_simulator->max_reorient_angles[1] - marginal_angle;
}

void PlanerPoseProblem::SetUnitReorientPushes( )
{
    // the reorient push angles should be less that the maximum reorient push angles by a marginal angle
    // knowing the step_angle, the total reorientation required to reach the goal orientation the CW and
    // CCW unit push vectors are computed
    
    float marginal_angle = 10 * M_PI/180;

    int num_step_pushes = m_simulator->delta_angle[0] / m_simulator->step_angle[0];
    float last_push_angle = fmod( m_simulator->delta_angle[0] , m_simulator->step_angle[0] );
    printf( " num_step_pushes = %d last_push_angle = %f \n", num_step_pushes,last_push_angle);
    m_simulator->num_reorient_CW = num_step_pushes + 1;
    
    // calculate fence direction for the first push by rotating v2v1 by step angle size
    
    //calculating the sin and cos of theta
    float ctheta = glm::cos( m_simulator->step_angle[0] );
    float stheta = glm::sin( m_simulator->step_angle[0] ); 
    
    glm::mat2 Rotation_CW = glm::mat2( ctheta, stheta,  //first column
                                      -stheta, ctheta);  //second column 
    // v2v1 roatated my step angle to get the fence direction
    glm::vec2 fence_vector = m_simulator->edge_vectors[0] * Rotation_CW;
    m_simulator->unit_reorient_CW.push_back( glm::normalize( glm::vec2( -fence_vector[1] , fence_vector[0] )) );
    m_simulator->mag_reorient_CW.push_back( 1.0 );
    
    int i = 0;

        printf("CW 0 %f  %f\n",m_simulator->unit_reorient_CW[i][0],m_simulator->unit_reorient_CW[i][1]);
    for( i = 1 ; i < num_step_pushes; ++i)
    {
        m_simulator->unit_reorient_CW.push_back( glm::normalize( m_simulator->unit_reorient_CW[i-1] * Rotation_CW ) );
        m_simulator->mag_reorient_CW.push_back( 1.0 );
        printf("CW 0 %f  %f\n",m_simulator->unit_reorient_CW[i][0],m_simulator->unit_reorient_CW[i][1]);
    }

    ctheta = glm::cos( last_push_angle );
    stheta = glm::sin( last_push_angle );

    Rotation_CW = glm::mat2( ctheta, stheta,  //first column
                             -stheta, ctheta);  //second column
    m_simulator->unit_reorient_CW.push_back( glm::normalize( m_simulator->unit_reorient_CW[i-1] * Rotation_CW ) );
    m_simulator->mag_reorient_CW.push_back( 1.0 );
        printf("CW 0 %f  %f\n",m_simulator->unit_reorient_CW[i][0],m_simulator->unit_reorient_CW[i][1]);



    //calculating the unit CCW reorient pushes
    num_step_pushes = m_simulator->delta_angle[1] / m_simulator->step_angle[1];
    last_push_angle = fmod( m_simulator->delta_angle[1], m_simulator->step_angle[1] );
   
    m_simulator->num_reorient_CCW = num_step_pushes + 1;
    
    ctheta = glm::cos( m_simulator->step_angle[1] );
    stheta = glm::sin( m_simulator->step_angle[1] );
    glm::mat2 Rotation_CCW = glm::mat2( ctheta,-stheta,  //first column
                                        stheta, ctheta);  //second column 
    // v1v2 rotated by step angle to get the fence direction
    fence_vector = (-1.0f*m_simulator->edge_vectors[0]) * Rotation_CCW;
    m_simulator->unit_reorient_CCW.push_back( glm::normalize( glm::vec2( fence_vector[1] , -fence_vector[0] )) );
    m_simulator->mag_reorient_CCW.push_back( 1.0 );
        printf("CCW 0 %f  %f\n",m_simulator->unit_reorient_CCW[0][0],m_simulator->unit_reorient_CCW[0][1]);

    for( i = 1 ; i < num_step_pushes; ++i)
    {
        m_simulator->unit_reorient_CCW.push_back( glm::normalize( m_simulator->unit_reorient_CCW[i-1] * Rotation_CCW ) );
        m_simulator->mag_reorient_CCW.push_back( 1.0 );
        printf("CCW 0 %f  %f\n",m_simulator->unit_reorient_CCW[i][0],m_simulator->unit_reorient_CCW[i][1]);
    }

    ctheta = glm::cos( last_push_angle );
    stheta = glm::sin( last_push_angle );
    Rotation_CCW = glm::mat2( ctheta,-stheta,  //first column
                              stheta, ctheta);  //second column 

    m_simulator->unit_reorient_CCW.push_back( glm::normalize( m_simulator->unit_reorient_CCW[i-1] * Rotation_CCW ) );
    m_simulator->mag_reorient_CCW.push_back( 1.0 );
    printf("CCW 0 %f  %f\n",m_simulator->unit_reorient_CCW[i][0],m_simulator->unit_reorient_CCW[i][1]);
}

float PlanerPoseProblem::AngleBetweenVectors( glm::vec2 v1, glm::vec2 v2 )
{
    return glm::acos( glm::dot(v1, v2) / ( glm::length(v1) * glm::length(v2) ) );
}

void PlanerPoseProblem::CalculatePeshkinDistance( )
{
    float a = 0.0;
    float c = 0.0;
    float Bi1 = 0.0;
    float Bi2 = 0.0;
    
    // a is the radius of the smallest circle centered at the center of mass that circumscribes the object
    // to calculate a the distance of COM to each vertex of the triangle is computed and the largest distance 
    // is set to be a
    float e0 = glm::length( m_simulator->edge_com_vectors[0]); // V2com
    float e1 = glm::length( m_simulator->edge_com_vectors[1]); // V1com
    float e2 = glm::length( m_simulator->edge_com_vectors[2]); // V0com
    
    a = glm::max( e0, e1 );
    a = glm::max( a, e2 );
     
    // c is the distance from the center of mass to the point of contact
    // in CW rotation the point of contact is always v2 
    c = e0; // V2com
    
    // quantity num_reorient_pushes-1 of the pushes will have a Bi1 of 10 degrees
    float marginal_angle = 10 * M_PI/180;
    Bi1 = marginal_angle;
    
    // to calculate Bi2 the perpandicular vector to V2V1 is calculated
    // and the angle between this vector and the ComV2 is derived
    glm::vec2 Temp = glm::vec2( m_simulator->edge_vectors[0][1], -1.0f *m_simulator->edge_vectors[0][0] );
    Bi2 = AngleBetweenVectors( Temp, -1.0f*m_simulator->edge_com_vectors[0] );
    
    // set the peshkin distance of num_reorient_pushes-1 in CW orientation
    int i = 0;
    float K1 = (a*a + c*c) / (2*c);
    float cosBi1 = glm::cos( Bi1 );
    float cosBi2 = glm::cos( Bi2 );
    float K2 = glm::log( (1 - cosBi2) / (1 + cosBi2) ) - glm::log( (1 - cosBi1) / (1 + cosBi1) );
    float mi = K1 * K2;

    for( i = 0; i < m_simulator->num_reorient_CW-1 ; ++i )
    {
        m_simulator->peshkin_CW.push_back( mi );
    }

    // the Bi1 for the last push will be different
    // get the last CW reorient push and negate it

    // get the angle of the last reorient push
    float last_angle = fmod( m_simulator->delta_angle[0] , m_simulator->step_angle[0] );
    float ctheta = glm::cos( last_angle );
    float stheta = glm::sin( last_angle );

    glm::mat2 Rotation = glm::mat2( ctheta, stheta,  //first column
                                    -stheta, ctheta);  //second column
    Temp = m_simulator->edge_vectors[0] * Rotation;
    Temp = glm::vec2( Temp[1],-1.0f*Temp[0] );
    Bi1 = AngleBetweenVectors( Temp, -1.0f*m_simulator->edge_com_vectors[0] );

    cosBi1 = glm::cos( Bi1 );
    K2 = glm::log( (1 - cosBi2) / (1 + cosBi2) ) - glm::log( (1 - cosBi1) / (1 + cosBi1) );
    mi = K1 * K2;
    m_simulator->peshkin_CW.push_back( mi );
     

    // c is the distance from the center of mass to the point of contact
    // in CCW rotation the point of contact is always v1 
    c = e1; // V1com
    
    // quantity num_reorient_pushes-1 of the pushes will have a Bi1 of 10 degrees
    Bi1 = marginal_angle;
    
    // to calculate Bi2 the perpandicular vector to -1*V2V1 is calculated
    // and the angle between this vector and the ComV1 is derived
    glm::vec2 v1v2 = -1.0f * m_simulator->edge_vectors[0];
    Temp = glm::vec2( -1.0f*v1v2[1], v1v2[0] );
    Bi2 = AngleBetweenVectors( Temp, -1.0f*m_simulator->edge_com_vectors[1] );

    // set the peshkin distance of num_reorient_pushes-1 in CCW orientation
    i = 0;
    K1 = (a*a + c*c) / (2*c);
    cosBi1 = glm::cos( Bi1 );
    cosBi2 = glm::cos( Bi2 );
    K2 = glm::log( (1 - cosBi2) / (1 + cosBi2) ) - glm::log( (1 - cosBi1) / (1 + cosBi1) );
    mi = K1 * K2;

    for( i = 0; i < m_simulator->num_reorient_CCW-1 ; ++i )
    {

        m_simulator->peshkin_CCW.push_back( mi );
    }

    // the Bi1 for the last push will be different
    // get the last CCW reorient push and negate it
    last_angle = fmod( m_simulator->delta_angle[1] , m_simulator->step_angle[1] );
    ctheta = glm::cos( last_angle );
    stheta = glm::sin( last_angle );

    Rotation = glm::mat2( ctheta, -stheta,  //first column
                          stheta, ctheta);  //second column
    Temp = (-1.0f*m_simulator->edge_vectors[0]) * Rotation;
    Temp = glm::vec2( -1.0f*Temp[1],Temp[0] );
    Bi1 = AngleBetweenVectors( Temp, -1.0f*m_simulator->edge_com_vectors[1] );

    cosBi1 = glm::cos( Bi1 );
    K2 = glm::log( (1 - cosBi2) / (1 + cosBi2) ) - glm::log( (1 - cosBi1) / (1 + cosBi1) );
    mi = K1 * K2;
    m_simulator->peshkin_CCW.push_back( mi );
}


void PlanerPoseProblem::populatePushes(void)
{
    //use M_PI for angles
    glm::vec2 temp = m_simulator->GetGoalCenter() - m_simulator->GetInitCenterOfMass();
    for( int i = 0 ; i < 10 ; i++ )
    {
        m_simulator->unit_pushes_CCW.push_back( glm::vec2( 0 , -1.5) );
        m_simulator->magnitudes_CCW.push_back( 1.0 );
        m_simulator->unit_pushes_CW.push_back( glm::vec2( 0 , -1.5) );
        m_simulator->magnitudes_CW.push_back( 1.0 );
    }

}

Move PlanerPoseProblem::PPPAlgorithm(bool CW_rotation)
{
    glm::vec2 push;
    float push_mag;
    
    // gets the next unit push vector and its magnitude
    if( CW_rotation == true )
    {
        push = m_simulator->unit_pushes_CW[m_simulator->push_index];
        push_mag = m_simulator->magnitudes_CW[m_simulator->push_index];
    }
    else
    {
        push = m_simulator->unit_pushes_CCW[m_simulator->push_index];
        push_mag = m_simulator->magnitudes_CCW[m_simulator->push_index];
    }
    
    // determines if the push is a translation push or a reorient push
    // translation pushes are in the direction of the unit normal vectors into the edges at the
    // current pose
    glm::vec2 normal_v2v1 = m_simulator->curr_triangle.vertices[1] - m_simulator->curr_triangle.vertices[2];
    normal_v2v1 = glm::normalize( normal_v2v1 );
    normal_v2v1 = glm::vec2( -normal_v2v1[1], normal_v2v1[0] );

    glm::vec2 normal_v2v0 = m_simulator->curr_triangle.vertices[0] - m_simulator->curr_triangle.vertices[2];
    normal_v2v0 = glm::normalize( normal_v2v0 );
    normal_v2v0 = glm::vec2( normal_v2v0[1], -normal_v2v0[0] );

    glm::vec2 normal_v1v0 = m_simulator->curr_triangle.vertices[1] - m_simulator->curr_triangle.vertices[0];
    normal_v1v0 = glm::normalize( normal_v1v0 );
    normal_v1v0 = glm::vec2( normal_v1v0[1], -normal_v1v0[0] );

    if( glm::normalize(push) == normal_v2v1 || glm::normalize(push) == normal_v2v0 || glm::normalize(push) == normal_v1v0 )
    {
        m_simulator->push_index++;;
        Move move = { push_mag*push[0], push_mag*push[1], 0 };
        return move;
    }
    //use M_PI for angles
    glm::vec2 temp = m_simulator->GetGoalCenter() - m_simulator->GetCurrCenterOfMass();
    temp = glm::normalize(temp);
    
    
    Move move = {temp[0], temp[1], 0};
    return move;
}
