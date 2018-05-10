//
//  Graphics.cpp
//  Graphics
//
//  Created by nusha mehmanesh on 4/6/18.
//  Copyright © 2018 nusha mehmanesh. All rights reserved.
//

#include "Graphics.hpp"
#include <iostream>
#include <GLUT/GLUT.h>
#include <OpenGL/OpenGL.h>
#include <glm/glm.hpp>

Graphics *m_graphics = NULL;

Graphics::Graphics(void)
{
    //m_bugAlgorithms = new BugAlgorithms(&m_simulator);
    
    // initial and goal poses are unspecified
    init_specified = false;
    goal_specified = false;
    CW_rotation = true;
    
    m_frames = 0;
    m_exportFrames = 0;
    m_run = false;
    
    m_planner = new PlanerPoseProblem(&m_simulator);
    
}

Graphics::~Graphics(void)
{
    //if(m_bugAlgorithms)
        //delete m_bugAlgorithms;
}

void Graphics::MainLoop(void)
{
    m_graphics = this;
    
    //create window
    int    argc = 1;
    char  *args = (char*)"args";
    glutInit(&argc, &args);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(600, 400);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Polygon Pushing Algorithm");
    
    
    //register callback functions
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutIdleFunc(NULL);
    glutTimerFunc(15, CallbackEventOnTimer, 0);
    glutKeyboardFunc(CallbackEventOnKeyPress);
    
    //enter main event loop
    glutMainLoop();
}

void Graphics::HandleEventOnKeyPress(unsigned char key, int x, int y)
{
    switch(key)
    {
            //when escape key is pushed exit
        case 27:
            exit(0);
        case 'p':
            m_run = !m_run;
    }
}

void Graphics::HandleEventOnTimer(void)
{
    if( init_specified == true && goal_specified == true)
    {
        
        if(m_run && !m_simulator.HasPolygonReachedGoal())
        {
            // get the move
            //struct Move move = m_planner->PPPAlgorithm(CW_rotation);
            
            //setting the center of mass and the theta based of the new move
            /*m_simulator.SetCurrCOM(m_simulator.GetCurrCenterOfMass()[0] +
                                   move.m_dx,
                                   m_simulator.GetCurrCenterOfMass()[1] +
                                   move.m_dy);
            m_simulator.SetCurrTheta(m_simulator.GetCurrtheta() + move.m_dtheta);
            
            m_simulator.SetVertices(move.m_dx, move.m_dy);
            */
            m_simulator.SetCurrCOM( m_simulator.com_coord[m_simulator.push_index][0],
                                    m_simulator.com_coord[m_simulator.push_index][1] );
            m_simulator.curr_triangle.vertices[0] = m_simulator.v0_coord[m_simulator.push_index];
            m_simulator.curr_triangle.vertices[1] = m_simulator.v1_coord[m_simulator.push_index];
            m_simulator.curr_triangle.vertices[2] = m_simulator.v2_coord[m_simulator.push_index];
            m_simulator.push_index++;
            if(m_exportFrames)
                ExportFrameAsImage();
            
        }
    }
    
}

void Graphics::SetInitPose( )
{
    //calculating the sin and cos of theta
    float ctheta = glm::cos( m_simulator.init_triangle.theta);
    float stheta = glm::sin( m_simulator.init_triangle.theta);
        
    double x = 0;
    double y = 0;
    
    glm::vec2 local(0.0);   
    glm::mat2 R = glm::mat2( ctheta, -stheta,  //first column
                             stheta,  ctheta);  //second column   

    for(int i = 0; i < 3; ++i)
    {
        local = m_simulator.init_triangle.vertices[i] - m_simulator.init_triangle.com;
        // using local parameters to apply the rotation then converting to global coordinates
        // and applying translation
        m_simulator.init_triangle.vertices[i] = local * R + m_simulator.init_triangle.com;
        m_simulator.curr_triangle.vertices[i] = m_simulator.init_triangle.vertices[i];

    }
}
void Graphics::SetGoalPose( )
{
    //calculating the sin and cos of theta
    float ctheta = glm::cos( m_simulator.goal_triangle.theta);
    float stheta = glm::sin( m_simulator.goal_triangle.theta);
        
    double x = 0;
    double y = 0;
    
    glm::vec2 local(0.0);   
    glm::mat2 R = glm::mat2( ctheta, -stheta,  //first column
                             stheta,  ctheta);  //second column   
    for(int i = 0; i < 3; ++i)
    {
        local = m_simulator.goal_triangle.vertices[i] - m_simulator.goal_triangle.com;
        // using local parameters to apply the rotation then converting to global coordinates
        // and applying translation
        m_simulator.goal_triangle.vertices[i] = local * R + m_simulator.goal_triangle.com;
    }
}

void Graphics::HandleEventInitGoalPose( double mouse_x, double mouse_y )
{
    if( init_specified == false && goal_specified == false )
    {
        //set the init pose center of mass
        m_simulator.SetCurrCOM(mouse_x,mouse_y);
        //m_simulator.SetCurrCOM(-9,7);
        
        m_simulator.init_triangle.com = glm::vec2( mouse_x, mouse_y );
        //m_simulator.init_triangle.com = glm::vec2(-9,7);
       m_simulator.init_triangle.theta = 3*M_PI/2;
        
        /* set the vertices V1 = (-1,0) V2 = (2.5,0) V3 = (0,1.5)
         * using COM = (0.5,0.5)
         * V1 = COM + (-0.5,1)
         * V2 = COM + (2,-0.5)
         * V3 = COM + (-1.5,-0.5)
         */
        
        //setting the first vertex
        m_simulator.init_triangle.vertices[0] = m_simulator.init_triangle.com + glm::vec2( -0.5, 1.0 );
        //setting the second vertex
        m_simulator.init_triangle.vertices[1] = m_simulator.init_triangle.com + glm::vec2( 2.0, -0.5);
        //setting the third vertex
        m_simulator.init_triangle.vertices[2] = m_simulator.init_triangle.com + glm::vec2( -1.5, -0.5 );
        SetInitPose( );
    /*    printf(" v0 = [  %f  %f  ] v1 = [ %f  %f  ] v2 = [ %f  %f  ]\n" ,m_simulator.init_triangle.vertices[0][0],
                m_simulator.init_triangle.vertices[0][1], m_simulator.init_triangle.vertices[1][0], m_simulator.init_triangle.vertices[1][1],
                m_simulator.init_triangle.vertices[2][0], m_simulator.init_triangle.vertices[2][1]);
        printf(" com x = %f , Com y = %f \n",m_simulator.init_triangle.com[0], m_simulator.init_triangle.com[1] );
      */  
        m_planner->SetEdgeVectors( );
        // set flag to refect the change
        init_specified = true;
    }
    else if( init_specified == true && goal_specified == false)
    {
        //set the goal pose center of mass
        m_simulator.SetGoalCenter(mouse_x, mouse_y, M_PI); 
        //m_simulator.SetGoalCenter( 10 , -8, M_PI);
        //setting the first vertex
        m_simulator.goal_triangle.vertices[0] = m_simulator.goal_triangle.com + glm::vec2( -0.5, 1.0 );
        //setting the second vertex
        m_simulator.goal_triangle.vertices[1] = m_simulator.goal_triangle.com + glm::vec2( 2.0, -0.5);
        //setting the third vertex
        m_simulator.goal_triangle.vertices[2] = m_simulator.goal_triangle.com + glm::vec2( -1.5, -0.5 );
        
        SetGoalPose( );
        // set flag to refect the change
        goal_specified = true;
      /*  printf(" v0 = [  %f  %f  ] v1 = [ %f  %f  ] v2 = [ %f  %f  ]\n" ,m_simulator.goal_triangle.vertices[0][0],
                m_simulator.goal_triangle.vertices[0][1], m_simulator.goal_triangle.vertices[1][0], m_simulator.goal_triangle.vertices[1][1],
                m_simulator.goal_triangle.vertices[2][0], m_simulator.goal_triangle.vertices[2][1]);
        printf(" com x = %f , Com y = %f \n",m_simulator.goal_triangle.com[0], m_simulator.goal_triangle.com[1] ); */
        m_planner->SetTotalOrientationChange( );
        m_planner->GetMaxReorientAngle( );
        m_planner->SetTotalOrientationChange( );
        m_planner->SetStepAngle( );
        //m_planner->CalcluateRadiusFunction( );
        m_planner->SetUnitReorientPushes( );
        m_planner->CalculatePeshkinDistance( );
        m_planner->CalculateCis( );
        m_planner->CalculateUnitNormals( );
        m_planner->LPSolutionCW( );
        m_planner->LPSolutionCCW( );
        //m_planner->PlanerPoseProblem::populatePushes( );
        if( CW_rotation )
        {
            m_planner->ProduceMovesCW( );
        }
        else
        {
            m_planner->ProduceMovesCCW( );
        }
    }
    else
    {
        printf("init and goal poses are already specified!\n");
    }
}

/* Args:
 * button = right/left/middle click
 * state = GLUT_UP or GLUT_DOWN indicating whether the callback was due to a release or press
 * x, y = window relative coordinates when the mouse button state changed
 */
void Graphics::CallbackEventOnMouse( int button, int state, int x, int y)
{
    // upon a mouse click we end up here and then HandleEventInitGoalPose
    // is called with the coordinates of the mouse click
    if( m_graphics && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
    {
        double mouse_x, mouse_y;
        MousePosition(x, y, &mouse_x, &mouse_y);
        m_graphics->HandleEventInitGoalPose(mouse_x , mouse_y);
        glutPostRedisplay();
        
    }
}


void Graphics::HandleEventOnDisplay(void)
{
    //draw triangle
    if( init_specified == true )
       //&& goal_specified == false)
    {
        // draw triangle at init pose
        glColor3f(0, 0, 1);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        
        glColor3f(0, 0, 1);
        glBegin(GL_TRIANGLES);
        for( int i = 0; i < 3; ++i)
        {
            glVertex2d(
                       m_simulator.curr_triangle.vertices[i][0],
                       m_simulator.curr_triangle.vertices[i][1]);
        }
        glEnd();

    }
    
    
    //if( init_specified == true &&
    if( goal_specified == true )
    {
        // draw triangle at goal pose
        glColor3f(0, 1, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        
        glColor3f(0, 1, 0);
        glBegin(GL_TRIANGLES);
        for( int i = 0; i < 3; ++i)
        {
            glVertex2d(
                       m_simulator.goal_triangle.vertices[i][0],
                       m_simulator.goal_triangle.vertices[i][1]);
        }
        glEnd();
    }
    
    glEnd();
  
}


void Graphics::DrawTriangle2D(double cx, double cy)
{

}


void Graphics::CallbackEventOnDisplay(void)
{
    if(m_graphics)
    {
        glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
        glClearDepth(1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glShadeModel(GL_SMOOTH);
        
        glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-22, 22, -14, 14, -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        
        m_graphics->HandleEventOnDisplay();
        
        glutSwapBuffers();
    }
}

void Graphics::CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
        m_graphics->HandleEventOnTimer();
        glutTimerFunc(120, CallbackEventOnTimer, id);
        glutPostRedisplay();
    }
}



void Graphics::CallbackEventOnKeyPress(unsigned char key, int x, int y)
{
    if(m_graphics)
        m_graphics->HandleEventOnKeyPress(key, x, y);
}


// converts the integer coordinates to double coordinates
void Graphics::MousePosition(int x, int y, double *mouse_x, double *mouse_y)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble mouse_z;
    
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    
    gluUnProject(winX, winY, winZ, modelview, projection, viewport, mouse_x, mouse_y, &mouse_z);
}

void Graphics::ExportFrameAsImage(void)
{
    char fname[50];
    sprintf(fname, "frames_%05d.ppm", m_frames++);
    ExportFrameAsImage(fname);
}

void Graphics::ExportFrameAsImage(const char fname[])
{
    
    const int width = glutGet(GLUT_WINDOW_WIDTH);
    const int height= glutGet(GLUT_WINDOW_HEIGHT);
    
    char *temp  = new char[3 * width * height];
    char *image = new char[3 * width * height];
    
    FILE *fp = fopen(fname, "w");
    
    printf("Writing %s\n", fname);
    
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, temp);
    
    int  a, b, row_sz = 3*width;
    // Reverse rows
    for(int i=0; i < height; i+=1)
    {
        for(int j=0; j < width; j+=1)
        {
            a = i*row_sz+3*j;
            b = (height-i-1)*row_sz+3*j;
            image[a]   = temp[b];
            image[a+1] = temp[b+1];
            image[a+2] = temp[b+2];
        }
    }
    fprintf(fp, "P6\n");
    fprintf(fp, "%i %i\n 255\n", width, height);
    fwrite(image, sizeof(char), 3 * width * height, fp);
    fclose(fp);
    delete[] temp;
    delete[] image;
}

int main(int argc, char **argv)
{
    if( argc != 2 )
    {
        printf("PlanerPoseProblem <1 for CW / 0 for CCW>\n");
        exit(0);
    } 
    
    Graphics graphics;
    if( atoi(argv[1]) == 0 )
    {
        graphics.CW_rotation = false;
    }
    graphics.MainLoop();
    
    return 0;
}

