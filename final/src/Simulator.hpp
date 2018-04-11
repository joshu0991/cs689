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
    bool ArePointsNear(std::pair<double, double> p1,
                       std::pair<double, double> p2) const
    {
        double dx = p2.first - p1.first;
        double dy = p2.second - p1.second;
        return (dx*dx + dy*dy < 0.2);
    }
    
    /**
     *@brief Get x-coordinate and y-coordinate of the polygon's current center of mass
     */
    std::pair<double, double> GetCurrCenterOfMass(void) const
    {
        return triangle->curr_Com;
    }
    
    /**
     *@brief Get x-coordinate and y-coordinate of the polygon's initial center of mass
     */
    std::pair<double, double> GetInitCenterOfMass(void) const
    {
        return triangle->init_Com;
    }
    
    /**
     *@brief Get x-coordinate of and y-coordinate the goal pose
     */
    std::pair<double, double> GetGoalCenter(void) const
    {
        return goalCenter;
    }
    
    /**
     *@brief Get the orientation of the goal pose
     */
    double GetGoaltheta(void) const
    {
        return goal_theta;
    }
    
    /**
     *@brief Get the orientation of the curr pose
     */
    double GetCurrtheta(void) const
    {
        return triangle->curr_theta;
    }
    

  /**
     *@brief Returns true iff the Triangle's center of mass is at the goal center and
     * the orientation of the triangle is equal to the goal orientaion.
     */
    bool HasPolygonReachedGoal(void) const
    {
        return (ArePointsNear(triangle->curr_Com, goalCenter) &&
                ArePointsNear(triangle->curr_vertices[0], goal_vertices[0]) &&
                ArePointsNear(triangle->curr_vertices[1], goal_vertices[1]) &&
                ArePointsNear(triangle->curr_vertices[2], goal_vertices[2]) );
        
    }
    
protected:
    /**
     *@brief Set Triangle's center of mass
     *
     *@param cx x position of center
     *@param cy y position of center
     */
    void SetCurrCOM(double cx, double cy)
    {
        triangle->curr_Com.first = cx;
        triangle->curr_Com.second = cy;
        
        path.push_back(triangle->curr_Com);
    }
    
    /**
     *@brief Set Triangle's orientation
     *
     *@param theta Triangle's current orientation
     */
    void SetCurrTheta(double theta)
    {
        triangle->curr_theta = theta;
        path_theta.push_back(triangle->curr_theta);
    }
    
    /**
     *@brief Set goal center
     *@param x x position of the goal center
     *@param y y position of the goal center
     *@param theta orientation of the goal pose
     */
    void SetGoalCenter(double x, double y,double theta )
    {
        goalCenter.first = x;
        goalCenter.second = y;
        goal_theta = theta;
        
    }
    
    /**
     *@brief Set the vertices of the polygon using the triangle's current
     * center of mass and theta
     */
    void SetVertices(double dx, double dy)
    {
        //calculating the sin and cos of theta
        double ctheta = cos(triangle->curr_theta);
        double stheta = sin(triangle->curr_theta);
        
        double x = 0;
        double y = 0;
        for(int i = 0; i < 3; ++i)
        {
            //converting the vertices to local coordinates
            x = triangle->curr_vertices[i].first - (triangle->curr_Com.first - dx);
            y = triangle->curr_vertices[i].second - (triangle->curr_Com.second - dy);
            
            // using local parameters to apply the rotation then converting to global coordinates
            // and applying translation
            triangle->curr_vertices[i].first = (ctheta * x) -
                                               (stheta * y) +
                                                triangle->curr_Com.first;
            
            triangle->curr_vertices[i].second = (stheta * x) +
                                                (ctheta * y) +
                                                triangle->curr_Com.second;
        }
    }

    

    /**
     *@brief Goal's position
     */
    std::pair<double, double> goalCenter;
    
    /**
     *@brief Goal's orientation
     */
    double goal_theta;
    
    /**
     *@brief Goal's vertices
     */
    std::vector<std::pair< double, double > > goal_vertices;
    
    /**
     *@brief For storing the history of Polygon positions
     */
    std::vector<std::pair<double, double> > path;
    
    /**
     *@brief For storing the history of Polygon's orientation
     */
    std::vector<double> path_theta;
    
    class Triangle
    {
    public:
        static std::shared_ptr< Triangle > getInstance()
        {
            static std::shared_ptr< Triangle > instance;
            if (instance.get() == NULL)
            {
                instance.reset(new Triangle);
            }
            return instance;
        }

    private:
        Triangle() :
            init_vertices(3),
            curr_vertices(3),
            init_Com(-20, 13),
            curr_Com(-20, 13),
            init_theta(0),
            curr_theta(0)
        {
        }
        
    private:
        friend class Simulator;
        friend class Graphics;
        
        // Initial and current positions of the triangle's vertices
        std::vector<std::pair< double, double > > init_vertices;
        std::vector<std::pair< double, double > > curr_vertices;
        
        // Initial and current positions of the triangle's center of mass
        std::pair<double, double> init_Com;
        std::pair<double, double> curr_Com;
        
        // the Initial and current orientation of the triangle
        double init_theta;
        double curr_theta;
    };
    
    std::shared_ptr< Triangle > triangle;
    
    friend class Graphics;
    friend class PlanerPoseProblem;
    
};

#endif /* Simulator_hpp */
