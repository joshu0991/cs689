//
//  Graphics.hpp
//  Graphics
//
//  Created by nusha mehmanesh on 4/6/18.
//  Copyright © 2018 nusha mehmanesh. All rights reserved.
//

#ifndef Graphics_hpp
#define Graphics_hpp

#include <stdio.h>
#include "Simulator.hpp"
#include "PlanerPoseProblem.hpp"

class Graphics
{
public:
    
    Graphics();
    
    /**
     *@brief Destroy window
     */
    ~Graphics(void);
    
    /**
     *@brief Main event loop
     */
    void MainLoop(void);
    
    bool CW_rotation;
protected:
    /**
     *@brief Perform simulation step
     */
    void HandleEventOnTimer(void);
    
    /**
     *@brief Main rendering function
     */
    void HandleEventOnDisplay(void);
    
    void SetInitPose(void);
    void SetGoalPose(void);
    /**
     *@brief Respond to event when left button is clicked
     *
     *@param mouse_x x-position
     *@param mouse_y y-position
     */
    void HandleEventInitGoalPose( double mouse_x, double mouse_y );
    
    /**
     *@brief Respond to key presses
     *
     *@param key key is pressed
     */
    void HandleEventOnKeyPress(unsigned char key, int x, int y);
    
    /**
     *@brief Draw Triangle
     *
     *@param cx x position of the center of mass
     *@param cy y position of the center of mass
     */
    void DrawTriangle2D(double cx, double cy);
    
    /**
     *@name GLUT callback functions
     *@{
     */
    static void CallbackEventOnDisplay(void);
    static void CallbackEventOnMouse(int button, int state, int x, int y);
    static void CallbackEventOnTimer(int id);
    static void CallbackEventOnKeyPress(unsigned char key, int x, int y);
    static void MousePosition(const int x, const int y, double *posX, double *posY);
    
    /**
     *@}
     */
    
    /**
     *@brief An instance of the simulator
     */
    Simulator m_simulator;
    PlanerPoseProblem *m_planner;
    
    /**
     *@brief Pointer to an instance of the polygon pushing algorithm
     */
    //BugAlgorithms *m_bugAlgorithms;
    
    /**
     *@brief flags to identify if the initial and goal poses are specified
     */
    bool init_specified = false;
    bool goal_specified = false;
    
    void ExportFrameAsImage(void);
    void ExportFrameAsImage(const char fname[]);
    
    int  m_frames;
    bool m_exportFrames;
    bool m_run;
};
#endif /* Graphics_hpp */
