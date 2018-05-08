//
//  Simulator.cpp
//  Graphics
//
//  Created by nusha mehmanesh on 4/6/18.
//  Copyright © 2018 nusha mehmanesh. All rights reserved.
//

#include "Simulator.hpp"

Simulator::Simulator( )

{
    for( int i=0 ; i < 3 ; ++i )
    {
//        init_triangle.vertices[i] = glm::vec2(0.0, 0.0);
        init_triangle.vertices.push_back(glm::vec2(0.0, 0.0));
        goal_triangle.vertices.push_back(glm::vec2(0.0, 0.0));
        curr_triangle.vertices.push_back(glm::vec2(0.0, 0.0));
    }

    init_triangle.com = glm::vec2(0.0, 0.0);
    goal_triangle.com = glm::vec2(0.0, 0.0);
    curr_triangle.com = glm::vec2(0.0, 0.0);
    e1_max_angle = glm::vec2(0.0, 0.0);
    delta_angle = glm::vec2(0.0, 0.0);
    step_angle = glm::vec2(0.0, 0.0);

    init_triangle.theta = 0.0;
    goal_triangle.theta = 0.0;
    curr_triangle.theta = 0.0;

    push_index = 0;

    for( int i = 0 ; i < 6 ; ++i )
    {
        local_minima.push_back(0.0);
    }
    
}

Simulator::~Simulator(void)
{
 //nothing for now
}
