#include "mujoco.h"
#include "mjdata.h"
#include "mjmodel.h"
#include "stdio.h"
#include <iostream>
#include <fstream>

using namespace std;

char error[1000];
mjModel* m;
mjData* d;

int main(void)
{
   ofstream data_file ("data.txt");
   //data_file.open("data.txt");

   mj_activate("/home/student/mjpro140/bin/mjkey.txt");

   // load model from file and check for errors
   m = mj_loadXML("/home/student/mjpro140/model/ur5/ur5.xml", NULL, error, 1000);

   if( !m )
   {
      printf("%s\n", error);
      return 1;
   }
   else
   {
        data_file <<"Model loaded, parsed & converted sucessfully\n"
                  <<endl<<"MODEL_PARAMETERS"<< endl
                  <<"Gen.Coordinates :"<< m->nq <<endl
                  <<"DOF's :"<< m->nv <<endl
                  <<"Bodys :"<< m->nbody <<endl
                  <<"Joints:"<< m->njnt <<endl
                  <<"Ctrl.IP:"<< m->nu <<endl<<endl;

        cout<<endl<<"MODEL_PARAMETERS"<< endl
                 <<"Gen.Coordinates :"<< m->nq <<endl
                 <<"DOF's :"<< m->nv <<endl
                 <<"Bodys :"<< m->nbody <<endl
                 <<"Joints:"<< m->njnt <<endl
                 <<"Ctrl.IP:"<< m->nu <<endl;
                 //<< m->name_bodyadr[1] << endl;
                            // << *(m->body_mass) <<endl;
    }

   //int=int; float=float; mjNum=double; mjtByte=unsigned char

   cout << m->nconmax<<endl;
   int color_size = 4;
   for (int mat_id = 0; mat_id < m->nmat; mat_id++)
   {
       cout <<m->names[m->name_matadr[mat_id]]<<"'s rgba: ";
       for (int color_id = 0; color_id < color_size; color_id++)
       cout<< m->mat_rgba[mat_id*color_size + color_id] << " ";
       cout<<endl;
   }
   cout<<endl;



   // make data corresponding to model
   d = mj_makeData(m);
  //d->xfrc_applied[6] = 1.002877;

   cout<<endl<<endl<<"DATA PARAMETERS:"<<endl
             <<"Sim. Time: "<< d->time << endl
             <<"Pot. Energy: "<< d->energy[1] << endl
             <<"Kin. Energy: "<< d->energy[2] << endl
             <<"Size of Data(d): "<<sizeof(*d) <<"Bytes"<<endl;

   mjtNum* con_force;
   const char *nm = "model.txt",
              *nam = "data_b4_simu.txt",
              *nma = "data_aftr_simu.txt";

   mj_printModel(m,nm);

   int steps = 0, objects_in_scene = 1; //Objects [FREE JOINTS not FIXED to the PLANE] in ur5.xml
   double v_control[8] = {0,0,0,0,0,0,0,0};
   double p_control[8] = {-1,-0.5,0.9,-3.14,-1.57,0.8,-0.025,0.025};//{-1.57,-1.45,1.45,-3.14,-1.46,0.8,-0.025,0.025};
   for(int c=0; c < m->njnt-objects_in_scene; c++)
   {
     d->ctrl[c+8] = p_control[c];
     d->ctrl[c+16] = v_control[c];
   }

    mj_printData(m, d, nam);

   // run simulation for 10 seconds

   while( d->time<20)
   {
       for(int e=0; e< m->njnt-objects_in_scene; e++)
       {
         d->ctrl[e] = d->qfrc_bias[e+(objects_in_scene*6)]; // 1 free joint adds 6 DOF's
       }

       mj_step(m, d);

      steps++;
      data_file <<endl <<"Step "<<steps <<": "<<endl;
      for(int cf=0; cf<d->ncon; cf++)
      {
          //mj_contactForce(m, d, cf,con_force);
          data_file <<"Contact "<<cf <<": "<<endl
                    <<"    Contact between geoms "<<d->contact[cf].geom1<<" & "<<d->contact[cf].geom2<<endl;
                    //<<"    Force: "<<*con_force<<endl<<endl;
      }
   }
   /*
   cout<<(m->name_bodyadr)[1]<<endl;
   for(int it=0;it<m->nnames;it++)
   {
        cout<<m->names[it];
        if(!m->names[it])
            cout<<" ";
   }
   cout<<endl;*/

   mj_printData(m, d, nma);

   for (int z=0; z< m->njnt-objects_in_scene; z++)
       data_file <<"Joint-"<< z << endl
                 <<"    Goal::Cu.State::SS.Error => "
                 << d->ctrl[z+8] <<"::"
                 << d->qpos[z+(objects_in_scene*7)] <<"::" // 1 free joint adds 7 nq's
                 << d->ctrl[z+8] - d->qpos[z+(objects_in_scene*7)]<<"radians"<< endl;

   for(int cc=0; cc<d->ncon; cc++)
       data_file <<endl<<"Contact "<<cc <<": "<<endl
                 <<"    Contact between geoms "<<d->contact[cc].geom1<<" & "<<d->contact[cc].geom2<<endl;
                 //<<"    Force: "<<*con_force<<endl<<endl;

   for(int cf=0; cf<d->ncon; cf++)
   {
       mj_contactForce(m, d, cf,con_force);
       cout <<endl<<"Contact "<<cf <<": "<<endl
                 <<"    Contact between geoms "<<d->contact[cf].geom1<<" & "<<d->contact[cf].geom2<<endl
                 <<"    Force: "<<*con_force<<endl<<endl;
   }


   // free model and data, deactivate
   mj_deleteData(d);
   mj_deleteModel(m);
   mj_deactivate();
   data_file.close();

   return 0;
}


/*
float **a = &m->geom_rgba;
//int **a = &m->body_parentid;
for (int i = 0; i < 1; i++)
{
    for (int j = 0; j < 8; j++)
    {
       // cout<< a[i][j]<< "  ";
       cout<< (&m->geom_rgba)[i][j]<< "  ";
    }
    cout<<endl;
}
   mjtNum **f = &d->qfrc_applied;
   cout<<"Ext. Force: "<<endl;
   for (int j = 0; j < 6; j++)
         cout<< f[0][j]<< "  ";
   cout<<endl;
   for (int j = 6; j < 12; j++)
         cout<< f[0][j]<< "  ";
    cout<<endl;
   mjtNum **p = &d->xpos;
   for (int i = 0; i < 1; i++)
   {
       cout<<"Body Position: "<<endl;
       for (int j = 0; j < (m->nbody * 3); j++)
       {
         cout<< p[i][j]<< "  ";
       }
       cout<<endl;
   }

   int color_size = 4;
   for (int body_id = 0; body_id < m->nbody; body_id++)
   {
       cout <<"Body_"<<body_id<<" rgba: ";
       for (int color_id = 0; color_id < color_size; color_id++)
       cout<< m->geom_rgba[body_id*color_size + color_id] << " ";
       cout<<endl;
   }
   cout<<endl;

   d->ctrl[0] = 0.0;
   d->ctrl[1] = -1.42;
   d->ctrl[2] = 1.45;
   d->ctrl[3] = -3.14;
   d->ctrl[4] = 1.46;
   d->ctrl[5] = 0.0;
   d->ctrl[6] = -0.025;
   d->ctrl[7] = 0.025;

      cout <<endl<<"STEP:"<< steps <<endl<<"Force_Sensor: ";
      for (int j = 0; j < 3; j++)
      {
        cout<< d->sensordata[0*3 + j] << "  ";
      }
      cout<<endl;

      for (int i=1; i < m->nbody; i++)
      {
          cout<<"BODY_ID: "<<i<<endl<<"Position:";
          for (int j = 0; j < 3; j++)
          {
            cout<< d->xpos[i*3 + j] << "  ";
          }
          cout<<endl;

          cout<<"Cacc: ";
          for (int j = 0; j < 6; j++)
          {
            cout<< d->cacc[i*6 + j] << "  ";
          }
          cout<<endl;

          cout<<"Cfrc_int: ";
          for (int j = 0; j < 6; j++)
          {
            cout<< d->cfrc_int[i*6 + j] << "  ";
          }
          cout<<endl;

          cout<<"Cfrc_ext: ";
          for (int j = 0; j < 6; j++)
          {
            cout<< d->cfrc_ext[i*6 + j] << "  ";
          }
          cout<<endl;
      }
 */



