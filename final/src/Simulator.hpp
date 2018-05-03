//
//  Simulator.hpp
//  Graphics
//
//  Created by nusha mehmanesh on 4/6/18.
//  Copyright © 2018 nusha mehmanesh. All rights reserved.
//

#ifndef Simulator_hpp
#define Simulator_hpp

#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <glm/glm.hpp>


/**
 *@brief Simulate robot motion and sensor
 *
 *@remark
 *  ::Simulator provides functionality to simulate the robot motion and the sensor.
 *    Simulator is considered as an input to your implementation of
 *    the bug algorithm. As such, you should not make changes to ::Simulator.
 *    In particular, your bug algorithm should only access the public functions
 *    of ::Simulator.
 */
class Simulator
{
public:
    /**
     *@brief Initialize variables
     */
    Simulator(void);
    
    /**
     *@brief Delete allocated memory
     */
    ~Simulator(void);
    
    /**
     *@brief Determine if two points (x1, y1) and (x2, y2) are near each other
     */
    bool ArePointsNear( glm::vec2 p1,
                        glm::vec2 p2) const
    {

       float magnitude = glm::length( p2 - p1 );
       return (magnitude * magnitude < 0.2);
    }
    
    /**
     *@brief Get x-coordinate and y-coordinate of the polygon's current center of mass
     */
    glm::vec2  GetCurrCenterOfMass(void) const
    {
        return curr_triangle.com;
    }
    
    /**
     *@brief Get x-coordinate and y-coordinate of the polygon's initial center of mass
     */
    glm::vec2  GetInitCenterOfMass(void) const
    {
        return init_triangle.com;
    }
    
    /**
     *@brief Get x-coordinate of and y-coordinate the goal pose
     */
    glm::vec2 GetGoalCenter(void) const
    {
        return goal_triangle.com;
    }
    
    /**
     *@brief Get the orientation of the goal pose
     */
    float GetGoaltheta(void) const
    {
        return goal_triangle.theta;
    }
    
    /**
     *@brief Get the orientation of the curr pose
     */
    float GetCurrtheta(void) const
    {
        return curr_triangle.theta;
    }
    

  /**
     *@brief Returns true iff the Triangle's center of mass is at the goal center and
     * the orientation of the triangle is equal to the goal orientaion.
     */
    bool HasPolygonReachedGoal(void) const
    {
        return (ArePointsNear(curr_triangle.com, goal_triangle.com) &&
                ArePointsNear(curr_triangle.vertices[0], goal_triangle.vertices[0]) &&
                ArePointsNear(curr_triangle.vertices[1], goal_triangle.vertices[1]) &&
                ArePointsNear(curr_triangle.vertices[2], goal_triangle.vertices[2]) );
        
    }
    
protected:
    /**
     *@brief Set Triangle's center of mass
     *
     *@param cx x position of center
     *@param cy y position of center
     */
    void SetCurrCOM(float  cx, float cy)
    {
        curr_triangle.com = glm::vec2( cx, cy );
    }
    
    /**
     *@brief Set Triangle's orientation
     *
     *@param theta Triangle's current orientation
     */
    void SetCurrTheta(float theta)
    {
        curr_triangle.theta = theta;
    }
    
    /**
     *@brief Set goal center
     *@param x x position of the goal center
     *@param y y position of the goal center
     *@param theta orientation of the goal pose
     */
    void SetGoalCenter(float x, float y,float theta )
    {
        goal_triangle.com = glm::vec2( x, y );
        goal_triangle.theta = theta;
    }
    
    /**
     *@brief Set the vertices of the polygon using the triangle's current
     * center of mass and theta
     */
    void SetVertices(float dx, float dy)
    {
        //calculating the sin and cos of theta
        float ctheta = glm::cos(curr_triangle.theta);
        float stheta = glm::sin(curr_triangle.theta);
        
        glm::vec2 local = glm::vec2(0.0);
        glm::mat2 R = glm::mat2( ctheta, -stheta,  //first column
                                 stheta,  ctheta);  //second column

        for(int i = 0; i < 3; ++i)
        {
            //converting the vertices to local coordinates
            local = curr_triangle.vertices[i] - ( curr_triangle.com - glm::vec2( dx, dy ) );
            curr_triangle.vertices[i] = local * R + curr_triangle.com;
        }
    }

    

 public:
    typedef struct Triangle
    {
        // Positions of the triangle's vertices
        std::vector<glm::vec2> vertices;
        // Center of mass of the triangle
        glm::vec2 com;
        // Orientation of the vertices with respect to the center of mass
        float theta; 

    }Triangle;
    
    //std::shared_ptr< Triangle > init_triangle;
    Triangle init_triangle;
    Triangle goal_triangle;
    Triangle curr_triangle;

    std::vector<float> local_minima;
    glm::vec2 e1_max_angle;
    glm::vec2 delta_angle;
    glm::vec2 step_angle;

    std::vector<glm::vec2> unit_reorient_CW;
    std::vector<glm::vec2> unit_reorient_CCW;

    std::vector<float> mag_reorient_CW;
    std::vector<float> mag_reorient_CCW;
    
    friend class Graphics;
    friend class PlanerPoseProblem;
    
};

#endif /* Simulator_hpp */
