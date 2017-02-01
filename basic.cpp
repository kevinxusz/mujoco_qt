//-----------------------------------//
//  This file is part of MuJoCo.     //
//  Copyright (C) 2016 Roboti LLC.   //
//-----------------------------------//


#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <iostream>
#include <fstream>

using namespace std;


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=2 )
    {
        printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    ofstream op_file ("data_frm_basic.txt");

    // activate software
    mj_activate("/home/student/mjpro140/bin/mjkey.txt");

    int steps = 0,objects_in_scene = 1; //Objects [FREE JOINTS not FIXED to the PLANE] in ur5.xml

    // load and compile model
    char error[1000] = "Could not load binary model";
    if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
        m = mj_loadModel(argv[1], 0, 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);

    if( !m )
    {
        mju_error_s("Load model error: %s", error);
        op_file <<error;
    }
    else
    {
        op_file <<"Model loaded, parsed & converted sucessfully\n"
                <<endl<<"MODEL_PARAMETERS"<< endl
                <<"Gen.Coordinates :"<< m->nq <<endl
                <<"DOF's :"<< m->nv <<endl
                <<"Bodys :"<< m->nbody <<endl
                <<"Joints:"<< m->njnt <<endl
                <<"Ctrl.IP:"<< m->nu <<endl
                <<"No.of Free Objects:"<< objects_in_scene <<endl<<endl;

        cout<<endl<<"MODEL_PARAMETERS"<< endl
                  <<"Gen.Coordinates :"<< m->nq <<endl
                  <<"DOF's :"<< m->nv <<endl
                  <<"Bodys :"<< m->nbody <<endl
                  <<"Joints:"<< m->njnt <<endl
                  <<"Ctrl.IP:"<< m->nu <<endl
                  <<"No.of Free Objects:"<< objects_in_scene <<endl<<endl;
    }

    // make data
    d = mj_makeData(m);
    mjtNum* con_force;
    //cout<<"No.of Contacts:"<<m->nconmax<<endl;

    double start_pose[8]    = {0,0,0,0,0,0,0,0};//{0.5,-0.1,0.2,-2.5,-1,0,-0.05,0.05};{0,0,0,0,0,0,0,0}
    double pos[8] = {-1,-0.5,0.9,-3.14,-1.57,0.8,-0.025,0.025};//{-1.2,-1.45,1.45,-3.14,-1.46,0.8,-0.025,0.025};
    double vel[8] = {0,0,0,0,0,0,0,0};
     /*double goal = 0.9;
    size_t steps = 100;
    size_t counter = 1;*/

    for(int c=0; c < m->njnt-objects_in_scene; c++)
    {
      d->ctrl[c+8] = pos[c];
      d->ctrl[c+16] = vel[c];
      d->qpos[c+(objects_in_scene*7)] = start_pose[c];
    }

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjv_makeScene(&scn, 1000);                   // space for 1000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_100);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.

        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 &&  d->time < 15)
        {

            for(int e=0; e< m->njnt-objects_in_scene; e++)
            {
              d->ctrl[e] = d->qfrc_bias[e+(objects_in_scene*6)];
            }

            mj_step(m, d);

            steps++;
            op_file <<endl <<"Step "<<steps <<": "<<endl;

            for(int cf=0; cf<d->ncon; cf++)
            {
                //mj_contactForce(m, d, cf,con_force);
                 op_file <<"Contact "<<cf <<": "<<endl
                         <<"    Contact between geoms "<<d->contact[cf].geom1<<" & "<<d->contact[cf].geom2<<endl;
                       //<<"    Force: "<<*con_force<<endl<<endl;
            }

        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    for (int z=0; z< m->njnt-objects_in_scene; z++)
    {
         cout  <<"Joint-"<< z << endl
               <<"    Goal::Cu.State::SS.Error => "
               << d->ctrl[z+8] <<"::"
               << d->qpos[z+(objects_in_scene*7)] <<"::"
               << d->ctrl[z+8] - d->qpos[z+(objects_in_scene*7)]<<"radians"<< endl;

         op_file  <<"Joint-"<< z << endl
                  <<"    Goal::Cu.State::SS.Error => "
                  << d->ctrl[z+8] <<"::"
                  << d->qpos[z+(objects_in_scene*7)] <<"::"
                  << d->ctrl[z+8] - d->qpos[z+(objects_in_scene*7)]<<"radians"<< endl;
    }

    // close GLFW, free visualization storage
    glfwTerminate();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();
    op_file.close();

    return 1;
}

/*if (counter <= steps)
{
  d->ctrl[0+8] = ((double)counter/(double)steps)*goal;
  //cout << "goal: " << d->ctrl[1] << endl;
  counter++;
}*/
