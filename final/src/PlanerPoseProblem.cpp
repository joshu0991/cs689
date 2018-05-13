//
//  PlanerPoseProblem.cpp
//  Graphics
//
//  Created by nusha mehmanesh on 4/6/18.
//  Copyright © 2018 nusha mehmanesh. All rights reserved.
//

#include "PlanerPoseProblem.hpp"
#include <glm/glm.hpp>
#include <glpk.h>
#include <stdlib.h>


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

    for( i = 1 ; i < num_step_pushes; ++i)
    {
        m_simulator->unit_reorient_CW.push_back( glm::normalize( m_simulator->unit_reorient_CW[i-1] * Rotation_CW ) );
        m_simulator->mag_reorient_CW.push_back( 1.0 );
    }

    ctheta = glm::cos( last_push_angle );
    stheta = glm::sin( last_push_angle );

    Rotation_CW = glm::mat2( ctheta, stheta,  //first column
                             -stheta, ctheta);  //second column
    m_simulator->unit_reorient_CW.push_back( glm::normalize( m_simulator->unit_reorient_CW[i-1] * Rotation_CW ) );
    m_simulator->mag_reorient_CW.push_back( 1.0 );



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

    for( i = 1 ; i < num_step_pushes; ++i)
    {
        m_simulator->unit_reorient_CCW.push_back( glm::normalize( m_simulator->unit_reorient_CCW[i-1] * Rotation_CCW ) );
        m_simulator->mag_reorient_CCW.push_back( 1.0 );
    }

    ctheta = glm::cos( last_push_angle );
    stheta = glm::sin( last_push_angle );
    Rotation_CCW = glm::mat2( ctheta,-stheta,  //first column
                              stheta, ctheta);  //second column 

    m_simulator->unit_reorient_CCW.push_back( glm::normalize( m_simulator->unit_reorient_CCW[i-1] * Rotation_CCW ) );
    m_simulator->mag_reorient_CCW.push_back( 1.0 );
}

float PlanerPoseProblem::AngleBetweenVectors( glm::vec2 v1, glm::vec2 v2 )
{
    return glm::acos( glm::dot(v1, v2) / ( glm::length(v1) * glm::length(v2) ) );
}

glm::mat2 PlanerPoseProblem::GetRotationMatrix( float angle, bool CW )
{
    float ctheta = glm::cos( angle );
    float stheta = glm::sin( angle );

    if( CW )
    {
        return glm::mat2( ctheta, stheta,  //first column
                         -stheta, ctheta);  //second column 
    }
    
    return glm::mat2( ctheta,-stheta,  //first column
                      stheta, ctheta);  //second column 
}

void PlanerPoseProblem::CalculateCis( )
{
    std::vector<glm::vec2> COM_CW;
    std::vector<glm::vec2> COM_CCW;

    // the initial com is given
    COM_CW.push_back( m_simulator->init_triangle.com - m_simulator->init_triangle.vertices[2]);
    COM_CCW.push_back( m_simulator->init_triangle.com - m_simulator->init_triangle.vertices[1]);
    
    glm::mat2 Rotation = GetRotationMatrix( m_simulator->step_angle[0], true );

    int i = 0;
    for( i = 1 ; i < m_simulator->num_reorient_CW ; ++i)
    {
        COM_CW.push_back( COM_CW[i-1] * Rotation );
        m_simulator->Cis_CW.push_back( COM_CW[i] - COM_CW[i-1] );
    }
    Rotation = GetRotationMatrix( fmod(m_simulator->delta_angle[0],  m_simulator->step_angle[0]) , true );
    COM_CW.push_back( COM_CW[i-1] * Rotation );
    m_simulator->Cis_CW.push_back( COM_CW[i] - COM_CW[i-1] );

    
    Rotation = GetRotationMatrix( m_simulator->step_angle[1], false );

    for( i = 1 ; i < m_simulator->num_reorient_CCW ; ++i)
    {
        COM_CCW.push_back( COM_CCW[i-1] * Rotation );
        m_simulator->Cis_CCW.push_back( COM_CCW[i] - COM_CCW[i-1] );
    }

    Rotation = GetRotationMatrix( fmod(m_simulator->delta_angle[1],  m_simulator->step_angle[1]) , false );
    COM_CCW.push_back( COM_CCW[i-1] * Rotation );
    m_simulator->Cis_CCW.push_back( COM_CCW[i] - COM_CCW[i-1] );


}

void PlanerPoseProblem::CalculateUnitNormals( )
{
    // calculating the inward unit normals to the edges

    // inward unit normal of edge V2V1
    glm::vec2 ni1 = glm::vec2( -1.0f * m_simulator->edge_vectors[0][1], m_simulator->edge_vectors[0][0] );
    ni1 = glm::normalize( ni1 );
    
    // inward unit normal of edge V2V0
    glm::vec2 ni2 = glm::vec2( m_simulator->edge_vectors[1][1], -1.0f * m_simulator->edge_vectors[1][0] );
    ni2 = glm::normalize( ni2 );

    //inward unit normal of edge V0V1
    glm::vec2 ni3 = glm::vec2(  m_simulator->edge_vectors[2][1], -1.0f * m_simulator->edge_vectors[2][0] );
    ni3 = glm::normalize( ni3 );

    m_simulator->ni_CW.push_back( glm::mat3( glm::vec3(ni1, 0.0), glm::vec3(ni2, 0.0), glm::vec3(ni3, 0.0) ) );
    m_simulator->ni_CCW.push_back( glm::mat3( glm::vec3(ni1, 0.0), glm::vec3(ni2, 0.0), glm::vec3(ni3, 0.0) ) );
    
    glm::mat2 Rotation = GetRotationMatrix( m_simulator->step_angle[0] , true );

    int i = 0;
    for( i = 0 ; i < m_simulator->num_reorient_CW ; ++i )
    {
        if( i == (m_simulator->num_reorient_CW - 1) )
        {
            Rotation = GetRotationMatrix( fmod(m_simulator->delta_angle[0],  m_simulator->step_angle[0]) , true );
        }

        ni1 = m_simulator->ni_CW[i][0];
        ni2 = m_simulator->ni_CW[i][1];
        ni3 = m_simulator->ni_CW[i][2];

        ni1 = ni1 * Rotation;
        ni2 = ni2 * Rotation;
        ni3 = ni3 * Rotation;

        m_simulator->ni_CW.push_back( glm::mat3( glm::vec3(ni1, 0.0), glm::vec3(ni2, 0.0), glm::vec3(ni3, 0.0) ) );

    }

    Rotation = GetRotationMatrix( m_simulator->step_angle[1] , false );

    for( i = 0 ; i < m_simulator->num_reorient_CCW ; ++i )
    {
        if( i == (m_simulator->num_reorient_CCW - 1) )
        {
            Rotation = GetRotationMatrix( fmod(m_simulator->delta_angle[1],  m_simulator->step_angle[1]) , false );
        }

        ni1 = m_simulator->ni_CCW[i][0];
        ni2 = m_simulator->ni_CCW[i][1];
        ni3 = m_simulator->ni_CCW[i][2];

        ni1 = ni1 * Rotation;
        ni2 = ni2 * Rotation;
        ni3 = ni3 * Rotation;

        m_simulator->ni_CCW.push_back( glm::mat3( glm::vec3(ni1, 0.0), glm::vec3(ni2, 0.0), glm::vec3(ni3, 0.0) ) );

    }

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




void PlanerPoseProblem::LPSolutionCW( )
{
    // calculating T' = T - sum( Ci ) over all reorientions
    
    // T is the vector from the start center of mass to the goal center of mass
    glm::vec2 T = m_simulator->goal_triangle.com - m_simulator->init_triangle.com;

    // sum of all cis for CW
    glm::vec2 sum_cis = glm::vec2( 0.0, 0.0 );

    for( int i = 0 ; i < m_simulator->num_reorient_CW ; ++i)
    {
        sum_cis += m_simulator->Cis_CW[i];
    }

    glm::vec2 Tprime = T - sum_cis;

    // unit Tprime and perpandicular unit Tprime are computed
    glm::vec2 u_Tprime = glm::normalize( Tprime );
    glm::vec2 u_perp_Tprime = glm::vec2( -1.0f * u_Tprime[1], u_Tprime[0] );

    // calculating nij.Tprime and nij.Perp_Tprime
    std::vector<glm::vec3> coefficients1_aij;
    std::vector<glm::vec3> coefficients2_aij;
    
    int i = 0;
    
    glm::vec2 ni1;
    glm::vec2 ni2;
    glm::vec2 ni3;
    
    glm::vec3 ni;

    for( i = 0 ; i < m_simulator->num_reorient_CW+1 ; ++i )
    {
        ni1 = m_simulator->ni_CW[i][0];
        ni2 = m_simulator->ni_CW[i][1];
        ni3 = m_simulator->ni_CW[i][2];
        ni = glm::vec3( glm::dot( ni1 , u_Tprime ),glm::dot( ni2 , u_Tprime ), glm::dot( ni3 , u_Tprime ) );
        coefficients1_aij.push_back( ni );
        
        ni = glm::vec3( glm::dot( ni1 , u_perp_Tprime ),glm::dot( ni2 , u_perp_Tprime ), glm::dot( ni3 , u_perp_Tprime ) );
        coefficients2_aij.push_back( ni );
            
    }
    
    // calculating pi.Tprime and pi.Perp_Tprime
    std::vector<float> coefficients1_bi;
    std::vector<float> coefficients2_bi;


    for( i = 0 ; i < m_simulator->num_reorient_CW ; ++i )
    {
        coefficients1_bi.push_back( glm::dot( m_simulator->unit_reorient_CW[i], u_Tprime ) ); 
        coefficients2_bi.push_back( glm::dot( m_simulator->unit_reorient_CW[i], u_perp_Tprime ) ); 
    }
    
    float Tprime_mag = glm::length( Tprime );

    // setting up LP solver
    
    glp_prob *lp; 
    // creates a problem object, the object is initially empty
    lp = glp_create_prob();
    // assigns a symbolic name to the problem
    glp_set_prob_name( lp, "CW_LP_solver" );
    // calls the routine glm_set_obj_dir in order to set the optimization direction flag
    glp_set_obj_dir( lp, GLP_MIN );
    // the second arg is the number of rows added to the solver, #rows = #auxiliary variables =
    // #constraints the Problem is subjest to ( here 2 )
    glp_add_rows( lp, 2 );
    // assigns the symbolic name 'p' to the first row. here p is length of Tprime
    glp_set_row_name( lp, 1, "p" );
    // sets the type and bounds of the first row. here the row is not bounded from above or 
    // below meaning we have an equality constraint so we use GLP_FX lb = x = ub Fixed variable
    // here ub is ignored void glp set row bnds(glp prob *P, int i, int type, double lb, double ub);
    glp_set_row_bnds( lp, 1, GLP_FX, Tprime_mag, 0.0 );
    // assigns the symbolic name 'q' to the secodn row. here p is 0.0 
    glp_set_row_name( lp, 2, "q" );
    glp_set_row_bnds( lp, 2, GLP_FX, 0.0, 0.0 );
    
    // calculating the number of variables/columns to provide as an input to the solver
    float num_col = ((m_simulator->num_reorient_CW + 1) * 3) + m_simulator->num_reorient_CW;
    // adds num_col columns to the problem object
    glp_add_cols( lp, num_col );
    
    // column names will be in the following format a00_0 b00
    char col_name_a[6] = {'a','0','0','_','0',0}; 
    char col_name_b[4] = {'b','0','0',0};
    for( i = 0 ; i < m_simulator->num_reorient_CW + 1 ; ++i )
    {   
        //naming the aijs separate
        // for ai1
        col_name_a[4] = '1';
        if( i < 10 )
        {
            col_name_a[2] = '0' + i;
        }
        else if( i >= 10 )
        {
            col_name_a[1] = '0' + (i/10);
            col_name_a[2] = '0' + (i%10);
        }
        // assigns the symbolic name a00_0 to the column variable
        glp_set_col_name( lp, 3*i+1, col_name_a );
        // sets the type and bounds of the column GLP_LO means that the column has a lower bound
        glp_set_col_bnds( lp, 3*i+1, GLP_LO, 0.0, 0.0 );
        // sets the objective coefficient for the column
        glp_set_obj_coef( lp, 3*i+1, 1.0 );
            
        // ai2
        col_name_a[4] = '2';
        glp_set_col_name( lp, 3*i+2, col_name_a );
        // sets the type and bounds of the column GLP_LO means that the column has a lower bound
        glp_set_col_bnds( lp, 3*i+2, GLP_LO, 0.0, 0.0 );
        // sets the objective coefficient for the column
        glp_set_obj_coef( lp, 3*i+2, 1.0 );

        // ai3
        col_name_a[4] = '3';
        glp_set_col_name( lp, 3*i+3, col_name_a );
        // sets the type and bounds of the column GLP_LO means that the column has a lower bound
        glp_set_col_bnds( lp, 3*i+3, GLP_LO, 0.0, 0.0 );
        // sets the objective coefficient for the column
        glp_set_obj_coef( lp, 3*i+3, 1.0 );

    }
    
    i = 3 * i + 1;

    for( int j = 0 ; j < m_simulator->num_reorient_CW  ; ++j )
    {
        //naming the bis separate
        if( j < 10 )
        {
            col_name_b[2] = '0' + j + 1;
        }
        else if( j >= 10 )
        {
            col_name_b[1] = '0' + ((j+1)/10);
            col_name_b[2] = '0' + ((j+1)%10);
        }
        // assigns the symbolic name a00_0 to the column variable
        glp_set_col_name( lp, i, col_name_b );
        // sets the type and bounds of the column GLP_LO means that the column has a lower bound
        glp_set_col_bnds( lp, i, GLP_LO, m_simulator->peshkin_CW[j], 0.0 );
        // sets the objective coefficient for the column
        glp_set_obj_coef( lp, i, 1.0 );
        i++;

    }
    
    // prepare non_zero elements of the constraint matrix (ie, constraint coefficients). row indices
    // of each element are stored in the array ia, column indices are stored in the array ja, and 
    // numerical values of corresponding elements are stored in the array ar.
    int ia[100+1], ja[100 + 1];
    double ar[100+1];

    for( i = 0 ; i < m_simulator->num_reorient_CW + 1 ; ++i )
    {
        ia[3*i+1] = 1;
        ja[3*i+1] = 3*i+1;
        ar[3*i+1] = coefficients1_aij[i][0];

        ia[3*i+2] = 1;
        ja[3*i+2] = 3*i+2;
        ar[3*i+2] = coefficients1_aij[i][1];

        ia[3*i+3] = 1;
        ja[3*i+3] = 3*i+3;
        ar[3*i+3] = coefficients1_aij[i][2];
        
    }

    i = 3 * i + 1;
    for(int j = 0 ; j < m_simulator->num_reorient_CW ; ++j)
    {
        ia[i] = 1;
        ja[i] = i;
        ar[i] = coefficients1_bi[j]; 
        i++;
    }
    
    i--;
    int j = 0;
    // second constraint 
    for(j = 0 ; j < m_simulator->num_reorient_CW+1 ; ++j)
    {
        ia[i+1] = 2;
        ja[i+1] = 3*j+1;
        ar[i+1] = coefficients2_aij[j][0];

        ia[i+2] = 2;
        ja[i+2] = 3*j+2;
        ar[i+2] = coefficients2_aij[j][1];

        ia[i+3] = 2;
        ja[i+3] = 3*j+3;
        ar[i+3] = coefficients2_aij[j][2];
        
        i+=3;
    }
    i++;
    j = 3*j + 1;
    for(int k = 0 ; k < m_simulator->num_reorient_CW ; ++k)
    {
        ia[i] = 2;
        ja[i] = j;
        ar[i] = coefficients2_bi[k];
        j++;
        i++;
    }
    
    // loads information from these three arrays into the problem object
    glp_load_matrix( lp, --i, ia, ja, ar );
    // all data have been entered into the problem object, and therefore the statement calls
    // the routine glp_simplex which is a driver to the simplex method in order to solve the
    // LP problem. this routine finds an optimal solution and stores all relevant information
    // back into the problem object.
    glp_simplex( lp, NULL );

    // gets a computed value of the objective function
    double z = glp_get_obj_val( lp );

    // obtain computed values of the structural variables(columns), which correspond
    // to the optimal basic solution found by the solver
    for( i = 1 ; i <= num_col ; ++i )
    {
        m_simulator->magnitudes_CW.push_back( glp_get_col_prim( lp, i ) );
        printf("CW magnitude of push %d = %f \n",i , m_simulator->magnitudes_CW[i-1]);
    }

    //frees all the memory allocated to the problem object
    glp_delete_prob( lp );

}


void PlanerPoseProblem::LPSolutionCCW( )
{
    // calculating T' = T - sum( Ci ) over all reorientions
    
    // T is the vector from the start center of mass to the goal center of mass
    glm::vec2 T = m_simulator->goal_triangle.com - m_simulator->init_triangle.com;

    // sum of all cis for CCW
    glm::vec2 sum_cis = glm::vec2( 0.0, 0.0 );

    for( int i = 0 ; i < m_simulator->num_reorient_CCW ; ++i)
    {
        sum_cis += m_simulator->Cis_CCW[i];
    }

    glm::vec2 Tprime = T - sum_cis;

    // unit Tprime and perpandicular unit Tprime are computed
    glm::vec2 u_Tprime = glm::normalize( Tprime );
    glm::vec2 u_perp_Tprime = glm::vec2( -1.0f * u_Tprime[1], u_Tprime[0] );


    // calculating nij.Tprime and nij.Perp_Tprime
    std::vector<glm::vec3> coefficients1_aij;
    std::vector<glm::vec3> coefficients2_aij;
    
    int i = 0;
    
    glm::vec2 ni1;
    glm::vec2 ni2;
    glm::vec2 ni3;
    
    glm::vec3 ni;

    for( i = 0 ; i < m_simulator->num_reorient_CCW+1 ; ++i )
    {
        ni1 = m_simulator->ni_CCW[i][0];
        ni2 = m_simulator->ni_CCW[i][1];
        ni3 = m_simulator->ni_CCW[i][2];
        ni = glm::vec3( glm::dot( ni1 , u_Tprime ),glm::dot( ni2 , u_Tprime ), glm::dot( ni3 , u_Tprime ) );
        coefficients1_aij.push_back( ni );
        
        ni = glm::vec3( glm::dot( ni1 , u_perp_Tprime ),glm::dot( ni2 , u_perp_Tprime ), glm::dot( ni3 , u_perp_Tprime ) );
        coefficients2_aij.push_back( ni );
            
    }
    
    // calculating pi.Tprime and pi.Perp_Tprime
    std::vector<float> coefficients1_bi;
    std::vector<float> coefficients2_bi;


    for( i = 0 ; i < m_simulator->num_reorient_CCW ; ++i )
    {
        coefficients1_bi.push_back( glm::dot( m_simulator->unit_reorient_CCW[i], u_Tprime ) ); 
        coefficients2_bi.push_back( glm::dot( m_simulator->unit_reorient_CCW[i], u_perp_Tprime ) ); 
    }
    
    float Tprime_mag = glm::length( Tprime );

    // setting up LP solver
    
    glp_prob *lp; 
    // creates a problem object, the object is initially empty
    lp = glp_create_prob();
    // assigns a symbolic name to the problem
    glp_set_prob_name( lp, "CCW_LP_solver" );
    // calls the routine glm_set_obj_dir in order to set the optimization direction flag
    glp_set_obj_dir( lp, GLP_MIN );
    // the second arg is the number of rows added to the solver, #rows = #auxiliary variables =
    // #constraints the Problem is subjest to ( here 2 )
    glp_add_rows( lp, 2 );
    // assigns the symbolic name 'p' to the first row. here p is length of Tprime
    glp_set_row_name( lp, 1, "p" );
    // sets the type and bounds of the first row. here the row is not bounded from above or 
    // below meaning we have an equality constraint so we use GLP_FX lb = x = ub Fixed variable
    // here ub is ignored void glp set row bnds(glp prob *P, int i, int type, double lb, double ub);
    glp_set_row_bnds( lp, 1, GLP_FX, Tprime_mag, 0.0 );
    // assigns the symbolic name 'q' to the secodn row. here p is 0.0 
    glp_set_row_name( lp, 2, "q" );
    glp_set_row_bnds( lp, 2, GLP_FX, 0.0, 0.0 );
    
    // calculating the number of variables/columns to provide as an input to the solver
    float num_col = ((m_simulator->num_reorient_CCW + 1) * 3) + m_simulator->num_reorient_CCW;
    // adds num_col columns to the problem object
    glp_add_cols( lp, num_col );
    
    // column names will be in the following format a00_0 b00
    char col_name_a[6] = {'a','0','0','_','0',0}; 
    char col_name_b[4] = {'b','0','0',0};
    for( i = 0 ; i < m_simulator->num_reorient_CCW + 1 ; ++i )
    {   
        //naming the aijs separate
        // for ai1
        col_name_a[4] = '1';
        if( i < 10 )
        {
            col_name_a[2] = '0' + i;
        }
        else if( i >= 10 )
        {
            col_name_a[1] = '0' + (i/10);
            col_name_a[2] = '0' + (i%10);
        }
        // assigns the symbolic name a00_0 to the column variable
        glp_set_col_name( lp, 3*i+1, col_name_a );
        // sets the type and bounds of the column GLP_LO means that the column has a lower bound
        glp_set_col_bnds( lp, 3*i+1, GLP_LO, 0.0, 0.0 );
        // sets the objective coefficient for the column
        glp_set_obj_coef( lp, 3*i+1, 1.0 );
            
        // ai2
        col_name_a[4] = '2';
        glp_set_col_name( lp, 3*i+2, col_name_a );
        // sets the type and bounds of the column GLP_LO means that the column has a lower bound
        glp_set_col_bnds( lp, 3*i+2, GLP_LO, 0.0, 0.0 );
        // sets the objective coefficient for the column
        glp_set_obj_coef( lp, 3*i+2, 1.0 );

        // ai3
        col_name_a[4] = '3';
        glp_set_col_name( lp, 3*i+3, col_name_a );
        // sets the type and bounds of the column GLP_LO means that the column has a lower bound
        glp_set_col_bnds( lp, 3*i+3, GLP_LO, 0.0, 0.0 );
        // sets the objective coefficient for the column
        glp_set_obj_coef( lp, 3*i+3, 1.0 );

    }
    
    i = 3 * i + 1;

    for( int j = 0 ; j < m_simulator->num_reorient_CCW  ; ++j )
    {
        //naming the bis separate
        if( j < 10 )
        {
            col_name_b[2] = '0' + j + 1;
        }
        else if( j >= 10 )
        {
            col_name_b[1] = '0' + ((j+1)/10);
            col_name_b[2] = '0' + ((j+1)%10);
        }
        // assigns the symbolic name a00_0 to the column variable
        glp_set_col_name( lp, i, col_name_b );
        // sets the type and bounds of the column GLP_LO means that the column has a lower bound
        glp_set_col_bnds( lp, i, GLP_LO, m_simulator->peshkin_CCW[j], 0.0 );
        // sets the objective coefficient for the column
        glp_set_obj_coef( lp, i, 1.0 );
        i++;

    }
    
    // prepare non_zero elements of the constraint matrix (ie, constraint coefficients). row indices
    // of each element are stored in the array ia, column indices are stored in the array ja, and 
    // numerical values of corresponding elements are stored in the array ar.
    int ia[100+1], ja[100 + 1];
    double ar[100+1];

    for( i = 0 ; i < m_simulator->num_reorient_CCW + 1 ; ++i )
    {
        ia[3*i+1] = 1;
        ja[3*i+1] = 3*i+1;
        ar[3*i+1] = coefficients1_aij[i][0];

        ia[3*i+2] = 1;
        ja[3*i+2] = 3*i+2;
        ar[3*i+2] = coefficients1_aij[i][1];

        ia[3*i+3] = 1;
        ja[3*i+3] = 3*i+3;
        ar[3*i+3] = coefficients1_aij[i][2];
        
    }

    i = 3 * i + 1;
    for(int j = 0 ; j < m_simulator->num_reorient_CCW ; ++j)
    {
        ia[i] = 1;
        ja[i] = i;
        ar[i] = coefficients1_bi[j]; 
        i++;
    }
    
    i--;
    int j = 0;
    // second constraint 
    for(j = 0 ; j < m_simulator->num_reorient_CCW+1 ; ++j)
    {
        ia[i+1] = 2;
        ja[i+1] = 3*j+1;
        ar[i+1] = coefficients2_aij[j][0];

        ia[i+2] = 2;
        ja[i+2] = 3*j+2;
        ar[i+2] = coefficients2_aij[j][1];

        ia[i+3] = 2;
        ja[i+3] = 3*j+3;
        ar[i+3] = coefficients2_aij[j][2];
        
        i+=3;
    }
    i++;
    j = 3*j + 1;
    for(int k = 0 ; k < m_simulator->num_reorient_CCW ; ++k)
    {
        ia[i] = 2;
        ja[i] = j;
        ar[i] = coefficients2_bi[k];
        j++;
        i++;
    }
    
    // loads information from these three arrays into the problem object
    glp_load_matrix( lp, --i, ia, ja, ar );
    // all data have been entered into the problem object, and therefore the statement calls
    // the routine glp_simplex which is a driver to the simplex method in order to solve the
    // LP problem. this routine finds an optimal solution and stores all relevant information
    // back into the problem object.
    glp_simplex( lp, NULL );

    // gets a computed value of the objective function
    double z = glp_get_obj_val( lp );

    // obtain computed values of the structural variables(columns), which correspond
    // to the optimal basic solution found by the solver
    for( i = 1 ; i <= num_col ; ++i )
    {
        m_simulator->magnitudes_CCW.push_back( glp_get_col_prim( lp, i ) );
        printf("CCW magnitude of push %d = %f \n",i , m_simulator->magnitudes_CCW[i-1]);
    }

    //frees all the memory allocated to the problem object
    glp_delete_prob( lp );
}

void PlanerPoseProblem::ProduceMovesCW( )
{
    glm::vec2 curr_com = m_simulator->init_triangle.com;
    glm::vec2 curr_v0 = m_simulator->init_triangle.vertices[0];
    glm::vec2 curr_v1 = m_simulator->init_triangle.vertices[1];
    glm::vec2 curr_v2 = m_simulator->init_triangle.vertices[2];

    // checks if a translation is performed on the init pose
    // if it is, it will get the intermediate poses during the translation
    if( m_simulator->magnitudes_CW[0] != 0.0 )
    {
        glm::vec2 push = m_simulator->ni_CW[0][0];
        float step_size = m_simulator->magnitudes_CW[0] / 10; 
        for( int i = 0 ; i < 10 ; i++ )
        {
            curr_com = curr_com + (step_size * push);
            curr_v0 = curr_v0 + (step_size * push);
            curr_v1 = curr_v1 + (step_size * push);
            curr_v2 = curr_v2 + (step_size * push);
            m_simulator->v0_coord.push_back( curr_v0 );
            m_simulator->v1_coord.push_back( curr_v1 );
            m_simulator->v2_coord.push_back( curr_v2 );
            m_simulator->com_coord.push_back( curr_com );
        }
    }
    if( m_simulator->magnitudes_CW[1] != 0.0 )
    {
        glm::vec2 push = m_simulator->ni_CW[0][1];
        float step_size = m_simulator->magnitudes_CW[1] / 10; 
        for( int i = 0 ; i < 10 ; i++ )
        {
            curr_com = curr_com + (step_size * push);
            curr_v0 = curr_v0 + (step_size * push);
            curr_v1 = curr_v1 + (step_size * push);
            curr_v2 = curr_v2 + (step_size * push);
            m_simulator->v0_coord.push_back( curr_v0 );
            m_simulator->v1_coord.push_back( curr_v1 );
            m_simulator->v2_coord.push_back( curr_v2 );
            m_simulator->com_coord.push_back( curr_com );
        }
    }
    if( m_simulator->magnitudes_CW[2] != 0.0 )
    {
        glm::vec2 push = m_simulator->ni_CW[0][2];
        float step_size = m_simulator->magnitudes_CW[2] / 10; 
        for( int i = 0 ; i < 10 ; i++ )
        {
            curr_com = curr_com + (step_size * push);
            curr_v0 = curr_v0 + (step_size * push);
            curr_v1 = curr_v1 + (step_size * push);
            curr_v2 = curr_v2 + (step_size * push);
            m_simulator->v0_coord.push_back( curr_v0 );
            m_simulator->v1_coord.push_back( curr_v1 );
            m_simulator->v2_coord.push_back( curr_v2 );
            m_simulator->com_coord.push_back( curr_com );
        }
    }
    glm::mat2 Rotation;
    int k = 3*m_simulator->num_reorient_CW + 3;
    // gets the intermediate poses for all other pushes
    for( int i = 0 ; i < m_simulator->num_reorient_CW ; ++i )
    {
        //getting intermediate poses for reorient pushes
        // get the peshkin distance for the reorient push
        float step_size_pi = m_simulator->peshkin_CW[i] / 10;
        float step_size_angle = 0.0; 
        if( i != m_simulator->num_reorient_CW-1 )
        {
            step_size_angle = m_simulator->step_angle[0]/10;
        }
        else
        {
            step_size_angle = fmod(m_simulator->delta_angle[0], m_simulator->step_angle[0])/ 10 ;
        }
        
        Rotation = GetRotationMatrix( step_size_angle, true );
        for( int j = 0 ; j < 10 ; j++ )
        {
            // rotate all vertices about the v2 axis
            curr_com = (curr_com - curr_v2) * Rotation + curr_v2;
            curr_v0 = (curr_v0 - curr_v2) * Rotation + curr_v2;
            curr_v1 = (curr_v1 - curr_v2) * Rotation + curr_v2;
            // apply translation
            curr_com = curr_com + step_size_pi*m_simulator->unit_reorient_CW[i];
            curr_v0 = curr_v0 + step_size_pi*m_simulator->unit_reorient_CW[i];
            curr_v1 = curr_v1 + step_size_pi*m_simulator->unit_reorient_CW[i];
            curr_v2 = curr_v2 + step_size_pi*m_simulator->unit_reorient_CW[i];
            
            m_simulator->v0_coord.push_back( curr_v0 );
            m_simulator->v1_coord.push_back( curr_v1 );
            m_simulator->v2_coord.push_back( curr_v2 );
            m_simulator->com_coord.push_back( curr_com );

        }
        // after the peshkin distance the triangle only translates
        // getting the magnitude of the reorient push
        float left_over = (m_simulator->magnitudes_CW[k] - m_simulator->peshkin_CW[i])/10;
        for( int j = 0 ; j < 10 ; j++ )
        {
            // apply translation
            curr_com = curr_com + left_over*m_simulator->unit_reorient_CW[i];
            curr_v0 = curr_v0 + left_over*m_simulator->unit_reorient_CW[i];
            curr_v1 = curr_v1 + left_over*m_simulator->unit_reorient_CW[i];
            curr_v2 = curr_v2 + left_over*m_simulator->unit_reorient_CW[i];
            
            m_simulator->v0_coord.push_back( curr_v0 );
            m_simulator->v1_coord.push_back( curr_v1 );
            m_simulator->v2_coord.push_back( curr_v2 );
            m_simulator->com_coord.push_back( curr_com );

        }

        // getting the intermediate pushes in the translation
        if( m_simulator->magnitudes_CW[3*(i+1)] != 0.0 )
        {
            glm::vec2 push = m_simulator->ni_CW[i+1][0];
            float step_size = m_simulator->magnitudes_CW[3*(i+1)] / 10; 
            for( int i = 0 ; i < 10 ; i++ )
            {
                curr_com = curr_com + (step_size * push);
                curr_v0 = curr_v0 + (step_size * push);
                curr_v1 = curr_v1 + (step_size * push);
                curr_v2 = curr_v2 + (step_size * push);
                m_simulator->v0_coord.push_back( curr_v0 );
                m_simulator->v1_coord.push_back( curr_v1 );
                m_simulator->v2_coord.push_back( curr_v2 );
                m_simulator->com_coord.push_back( curr_com );
            }
         }
        if( m_simulator->magnitudes_CW[3*(i+1)+1] != 0.0 )
        {
            glm::vec2 push = m_simulator->ni_CW[i+1][1];
            float step_size = m_simulator->magnitudes_CW[3*(i+1)+1] / 10; 
            for( int i = 0 ; i < 10 ; i++ )
            {
                 curr_com = curr_com + (step_size * push);
                curr_v0 = curr_v0 + (step_size * push);
                curr_v1 = curr_v1 + (step_size * push);
                curr_v2 = curr_v2 + (step_size * push);
                m_simulator->v0_coord.push_back( curr_v0 );
                m_simulator->v1_coord.push_back( curr_v1 );
                m_simulator->v2_coord.push_back( curr_v2 );
                m_simulator->com_coord.push_back( curr_com );
            }
        }
        if( m_simulator->magnitudes_CW[3*(i+1)+2] != 0.0 )
        {
            glm::vec2 push = m_simulator->ni_CW[i+1][2];
            float step_size = m_simulator->magnitudes_CW[3*(i+1)+2] / 10; 
            for( int i = 0 ; i < 10 ; i++ )
            {
                curr_com = curr_com + (step_size * push);
                curr_v0 = curr_v0 + (step_size * push);
                curr_v1 = curr_v1 + (step_size * push);
                curr_v2 = curr_v2 + (step_size * push);
                m_simulator->v0_coord.push_back( curr_v0 );
                m_simulator->v1_coord.push_back( curr_v1 );
                m_simulator->v2_coord.push_back( curr_v2 );
                m_simulator->com_coord.push_back( curr_com );
            }
        }
        
    }

}


void PlanerPoseProblem::ProduceMovesCCW( )
{
    glm::vec2 curr_com = m_simulator->init_triangle.com;
    glm::vec2 curr_v0 = m_simulator->init_triangle.vertices[0];
    glm::vec2 curr_v1 = m_simulator->init_triangle.vertices[1];
    glm::vec2 curr_v2 = m_simulator->init_triangle.vertices[2];

    // checks if a translation is performed on the init pose
    // if it is, it will get the intermediate poses during the translation
    if( m_simulator->magnitudes_CCW[0] != 0.0 )
    {
        glm::vec2 push = m_simulator->ni_CCW[0][0];
        float step_size = m_simulator->magnitudes_CCW[0] / 10; 
        for( int i = 0 ; i < 10 ; i++ )
        {
            curr_com = curr_com + (step_size * push);
            curr_v0 = curr_v0 + (step_size * push);
            curr_v1 = curr_v1 + (step_size * push);
            curr_v2 = curr_v2 + (step_size * push);
            m_simulator->v0_coord.push_back( curr_v0 );
            m_simulator->v1_coord.push_back( curr_v1 );
            m_simulator->v2_coord.push_back( curr_v2 );
            m_simulator->com_coord.push_back( curr_com );
        }
    }
    if( m_simulator->magnitudes_CCW[1] != 0.0 )
    {
        glm::vec2 push = m_simulator->ni_CCW[0][1];
        float step_size = m_simulator->magnitudes_CCW[1] / 10; 
        for( int i = 0 ; i < 10 ; i++ )
        {
            curr_com = curr_com + (step_size * push);
            curr_v0 = curr_v0 + (step_size * push);
            curr_v1 = curr_v1 + (step_size * push);
            curr_v2 = curr_v2 + (step_size * push);
            m_simulator->v0_coord.push_back( curr_v0 );
            m_simulator->v1_coord.push_back( curr_v1 );
            m_simulator->v2_coord.push_back( curr_v2 );
            m_simulator->com_coord.push_back( curr_com );
        }
    }
    if( m_simulator->magnitudes_CCW[2] != 0.0 )
    {
        glm::vec2 push = m_simulator->ni_CCW[0][2];
        float step_size = m_simulator->magnitudes_CCW[2] / 10; 
        for( int i = 0 ; i < 10 ; i++ )
        {
            curr_com = curr_com + (step_size * push);
            curr_v0 = curr_v0 + (step_size * push);
            curr_v1 = curr_v1 + (step_size * push);
            curr_v2 = curr_v2 + (step_size * push);
            m_simulator->v0_coord.push_back( curr_v0 );
            m_simulator->v1_coord.push_back( curr_v1 );
            m_simulator->v2_coord.push_back( curr_v2 );
            m_simulator->com_coord.push_back( curr_com );
        }
    }
    glm::mat2 Rotation;
    int k = 3*m_simulator->num_reorient_CCW + 3;
    // gets the intermediate poses for all other pushes
    for( int i = 0 ; i < m_simulator->num_reorient_CCW ; ++i )
    {
        //getting intermediate poses for reorient pushes
        // get the peshkin distance for the reorient push
        float step_size_pi = m_simulator->peshkin_CCW[i] / 10;
        float step_size_angle = 0.0; 
        if( i != m_simulator->num_reorient_CCW-1 )
        {
            step_size_angle = m_simulator->step_angle[1]/10;
        }
        else
        {
            step_size_angle = fmod(m_simulator->delta_angle[1], m_simulator->step_angle[1])/ 10 ;
        }
        
        Rotation = GetRotationMatrix( step_size_angle, false );
        for( int j = 0 ; j < 10 ; j++ )
        {
            // rotate all vertices about the v2 axis
            curr_com = (curr_com - curr_v1) * Rotation + curr_v1;
            curr_v0 = (curr_v0 - curr_v1) * Rotation + curr_v1;
            curr_v2 = (curr_v2 - curr_v1) * Rotation + curr_v1;
            // apply translation
            curr_com = curr_com + step_size_pi*m_simulator->unit_reorient_CCW[i];
            curr_v0 = curr_v0 + step_size_pi*m_simulator->unit_reorient_CCW[i];
            curr_v1 = curr_v1 + step_size_pi*m_simulator->unit_reorient_CCW[i];
            curr_v2 = curr_v2 + step_size_pi*m_simulator->unit_reorient_CCW[i];
            
            m_simulator->v0_coord.push_back( curr_v0 );
            m_simulator->v1_coord.push_back( curr_v1 );
            m_simulator->v2_coord.push_back( curr_v2 );
            m_simulator->com_coord.push_back( curr_com );

        }
        // after the peshkin distance the triangle only translates
        // getting the magnitude of the reorient push
        float left_over = (m_simulator->magnitudes_CCW[k] - m_simulator->peshkin_CCW[i])/10;
        for( int j = 0 ; j < 10 ; j++ )
        {
            // apply translation
            curr_com = curr_com + left_over*m_simulator->unit_reorient_CCW[i];
            curr_v0 = curr_v0 + left_over*m_simulator->unit_reorient_CCW[i];
            curr_v1 = curr_v1 + left_over*m_simulator->unit_reorient_CCW[i];
            curr_v2 = curr_v2 + left_over*m_simulator->unit_reorient_CCW[i];
            
            m_simulator->v0_coord.push_back( curr_v0 );
            m_simulator->v1_coord.push_back( curr_v1 );
            m_simulator->v2_coord.push_back( curr_v2 );
            m_simulator->com_coord.push_back( curr_com );

        }

        // getting the intermediate pushes in the translation
        if( m_simulator->magnitudes_CCW[3*(i+1)] != 0.0 )
        {
            glm::vec2 push = m_simulator->ni_CCW[i+1][0];
            float step_size = m_simulator->magnitudes_CCW[3*(i+1)] / 10; 
            for( int i = 0 ; i < 10 ; i++ )
            {
                curr_com = curr_com + (step_size * push);
                curr_v0 = curr_v0 + (step_size * push);
                curr_v1 = curr_v1 + (step_size * push);
                curr_v2 = curr_v2 + (step_size * push);
                m_simulator->v0_coord.push_back( curr_v0 );
                m_simulator->v1_coord.push_back( curr_v1 );
                m_simulator->v2_coord.push_back( curr_v2 );
                m_simulator->com_coord.push_back( curr_com );
            }
         }
        if( m_simulator->magnitudes_CCW[3*(i+1)+1] != 0.0 )
        {
            glm::vec2 push = m_simulator->ni_CCW[i+1][1];
            float step_size = m_simulator->magnitudes_CCW[3*(i+1)+1] / 10; 
            for( int i = 0 ; i < 10 ; i++ )
            {
                 curr_com = curr_com + (step_size * push);
                curr_v0 = curr_v0 + (step_size * push);
                curr_v1 = curr_v1 + (step_size * push);
                curr_v2 = curr_v2 + (step_size * push);
                m_simulator->v0_coord.push_back( curr_v0 );
                m_simulator->v1_coord.push_back( curr_v1 );
                m_simulator->v2_coord.push_back( curr_v2 );
                m_simulator->com_coord.push_back( curr_com );
            }
        }
        if( m_simulator->magnitudes_CCW[3*(i+1)+2] != 0.0 )
        {
            glm::vec2 push = m_simulator->ni_CCW[i+1][2];
            float step_size = m_simulator->magnitudes_CCW[3*(i+1)+2] / 10; 
            for( int i = 0 ; i < 10 ; i++ )
            {
                curr_com = curr_com + (step_size * push);
                curr_v0 = curr_v0 + (step_size * push);
                curr_v1 = curr_v1 + (step_size * push);
                curr_v2 = curr_v2 + (step_size * push);
                m_simulator->v0_coord.push_back( curr_v0 );
                m_simulator->v1_coord.push_back( curr_v1 );
                m_simulator->v2_coord.push_back( curr_v2 );
                m_simulator->com_coord.push_back( curr_com );
            }
        }
        
    }

}
