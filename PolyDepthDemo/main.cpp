#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include "PQP.h"
#include "MatVec.h"
#include "Stopwatch.h"
#include <iostream>
#include <fstream>
#include "C2A/LinearMath.h"
#include "C2A/C2A.h"
#include <PolyDepth/PolyDepth.h>
#include <PolyDepth/MeshObject.h>


#include "main.h"

int width=1280;
int height=960;

bool Flag_New=0;

StopwatchWin32 timer;
float timing;

int animating = 1;
int mode;
double beginx, beginy;
double dis = 1.0, azim = 0.0, elev = 0.0;
double ddis = 0.0, dazim = 0.0, delev = 0.0;

int step = 0;
int number_of_steps;

int query_type = 1;

double tolerance = .05;


//////////////
int* feature_types;
int* feature_ids;

///////////



// good
static const char* modelFile1 = "../tri_models/bunny_noholes.tri";
static const char* modelFile2 = "../tri_models/bunny_noholes.tri";

static const char* aniPath2 = "../models/bunny40k_1.ani";
static const char* aniPath1 = "../models/bunny40k_2.ani";

MeshObject *object1_, *object2_;
bool		use_motion_coherance_;
bool		use_maximal_clear_conf_;
//



void
init_viewer_window()
{
	GLfloat Ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };  
	GLfloat Diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };  
	GLfloat Specular[] = { 0.1f, 0.1f, 0.1f, 1.0f };   
	GLfloat SpecularExp[] = { 50 };              
	GLfloat Emission[] = { 0.1f, 0.1f, 0.1f, 1.0f };   

	glMaterialfv(GL_FRONT, GL_AMBIENT, Ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, Diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, Specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, SpecularExp);
	glMaterialfv(GL_FRONT, GL_EMISSION, Emission);

	glMaterialfv(GL_BACK, GL_AMBIENT, Ambient);
	glMaterialfv(GL_BACK, GL_DIFFUSE, Diffuse);
	glMaterialfv(GL_BACK, GL_SPECULAR, Specular);
	glMaterialfv(GL_BACK, GL_SHININESS, SpecularExp);
	glMaterialfv(GL_BACK, GL_EMISSION, Emission);

	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

	glEnable(GL_COLOR_MATERIAL);
	glutInitWindowSize(700, 700); 
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	glShadeModel(GL_FLAT);
	glClearColor(1, 1, 1, 0);
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-400,400,-400,400,-400,400);   
	//glOrtho(-15,15,-15,15,-15,15); 
	glMatrixMode(GL_MODELVIEW);

}

void
cb_mouse(int _b, int _s, int _x, int _y)
{
	if (_s == GLUT_UP)
	{
		dis += ddis;
		if (dis < .1) dis = .1;
		azim += dazim;
		elev += delev;
		ddis = 0.0;
		dazim = 0.0;
		delev = 0.0;
		return;
	}

	if (_b == GLUT_RIGHT_BUTTON)
	{
		mode = 0;
		beginy = _y;
		return;
	}
	else
	{
		mode = 1;
		beginx = _x;
		beginy = _y;
	}

}

void
cb_motion(int _x, int _y)
{
	if (mode == 0)
	{
		ddis = dis * (double)(_y - beginy)/0.5;
	}
	else
	{
		dazim = (_x - beginx)/5;
		delev = (_y - beginy)/5;      
	}  
	glutPostRedisplay();
}

void cb_keyboard(unsigned char key, int x, int y) 
{
	switch(key) 
	{
	case 'q': 

		delete object1_;
		delete object2_;
		exit(0);
	case 'm': use_motion_coherance_ = !use_motion_coherance_; break;
	case 'c': use_maximal_clear_conf_=!use_maximal_clear_conf_; break;
	case 'p': outputPerformance(timingFileName); break;
		default: {animating = 1 - animating;
		printf("NowFrame: %d\n",iframe);}
	}

	glutPostRedisplay();
}




void
BeginDraw()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity(); 
	glTranslatef(0.0, 0.0, -(dis+ddis));
	glRotated(elev+delev, 1.0, 0.0, 0.0);
	glRotated(azim+dazim, 0.0, 1.0, 0.0);
	glRotated(90.0,-1.0,0.0,0.0);
}

void
EndDraw()
{
	glFlush();
	glutSwapBuffers();
}




inline void glVertex3v(float V[3]) { glVertex3fv(V); }
inline void glVertex3v(double V[3]) { glVertex3dv(V); }


int nItr;


void
cb_display()//Display
{ 


	std::vector<Coord3D> local_penetration_depth;
	std::vector<Coord3D> local_penetration_features1;
	std::vector<Coord3D> local_penetration_features2;
	Coord3D global_penetration_depth;
	PolyDepthReturnValue result;

	BeginDraw();
	double oglm[16];
	if(iframe>=nframes)	{
		iframe = 0;
	}
	


	TransRT tr0;

	Transform trans0;
	Transform trans1;
	
	Transform trans01;

	Transform trans11;
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{		
			trans01.Rotation()[i][j]= xformframes[0][iframe].R[i][j];		
			trans11.Rotation()[i][j]= xformframes[1][iframe].R[i][j];
		}
	
		trans01.Translation()[i]= xformframes[0][iframe].T[i];	
		trans11.Translation()[i]= xformframes[1][iframe].T[i];
	}

	


	timer.Reset(); 
	timer.Start();

    nItr=0;

	for(int iTest=0;iTest<nTests;iTest++) 
	{ 			
		
		 
		Tri *t1 = object1_->c2a_model()->last_tri;
		Tri *t2 = object2_->c2a_model()->last_tri;
		result = 
			PolyDepth( 
			&trans01,object1_->c2a_model(), 
			&trans11, object2_->c2a_model(), 
			t1,t2,
			object1_->vertices(), object2_->vertices(),
			trans0, trans1, 
			local_penetration_depth,
			local_penetration_features1,
			local_penetration_features2,
			global_penetration_depth,
			nItr,
			object1_->clear_conf(),
			use_motion_coherance_,
			use_maximal_clear_conf_);

	}
	

	timer.Stop();
	tframes[iframe]=timer.GetTime()/nTests;
	GlobalPD[iframe] = global_penetration_depth;
	nIter[iframe]=nItr;


	glColor3f(1, 0,0);                 // setup color and transform
	MVtoOGL(oglm,xformframes[0][iframe].R,xformframes[0][iframe].T);
	glPushMatrix();
	glMultMatrixd(oglm);
	object1_->Draw();             // do gl rendering
	glPopMatrix();                    // restore transform

	//blue final A	



	glColor3f(0, 0,1);                 // setup color and transform
	MVtoOGL(oglm,xformframes[1][iframe].R,xformframes[1][iframe].T);
	glPushMatrix();
	glMultMatrixd(oglm);
	object2_->Draw();             // do gl rendering
	glPopMatrix();                    // restore transform

	//green- toc A
	glColor3f(0, 1, 0);	

	if (result.result_ == kNoPenetration) {
		object1_->clear_conf().AddClearConfiguration(trans01.Translation()-trans11.Translation(), result.distance_);
		
	}
	else if (result.result_ == kPenetration) {
		object1_->clear_conf().AddClearConfiguration(trans0.Translation()-trans11.Translation(), 0.00001);

		Transform2PQP(&trans0, tr0.R, tr0.T);
		glColor3f(1, 1, 0);                 // setup color and transform
		MVtoOGL(oglm,tr0.R,tr0.T);
		glPushMatrix();
		glMultMatrixd(oglm);
		object2_->Draw();
		glPopMatrix();
	}

	
 

  EndDraw();

  iframe++;
  if(animating)
	  glutPostRedisplay();
}

void main(int argc, char **argv)
{
  // init glut

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGBA | GLUT_STENCIL);

	glutInitWindowSize(720, 480);
	glutInitWindowPosition(550,400);
	glutCreateWindow("PolyDepth");

	// set OpenGL graphics state -- material props, perspective, etc.

	init_viewer_window();

	// set the callbacks

	glutDisplayFunc(cb_display);
	glutMouseFunc(cb_mouse);
	glutMotionFunc(cb_motion);  
	glutKeyboardFunc(cb_keyboard);

	object1_ = new MeshObject(modelFile1);
	object2_ = new MeshObject(modelFile2);
    use_motion_coherance_ = true;
	use_maximal_clear_conf_ = false;
	
    readXFormFrames(aniPath1, aniPath2);


	// print instructions
	printf("PolyDepth Demo - Falling:\n"
		"Press:\n"
		"c - using the motion coherence mode\n"
		"p - profiling into %s\n"
		"any other key to toggle animation on/off\n", timingFileName);

	// Enter the main loop.

	glutMainLoop();
}



void readXFormFrames( const char* filename1, const char* filename2 )
{
	ifstream fin1(filename1);
	ifstream fin2(filename2);
	float nf;
	float i;
	TransRT *xform;
	char c[5];

	fin1>>nf>>c;
	nframes=nf;
	while(!fin1.eof())
	{
		fin1>>i>>c;
		xform=&xformframes[0][(int)i];
		fin1>>xform->R[0][0]>>xform->R[1][0]>>xform->R[2][0]
			>>xform->R[0][1]>>xform->R[1][1]>>xform->R[2][1]
			>>xform->R[0][2]>>xform->R[1][2]>>xform->R[2][2];
		fin1>>xform->T[0]>>xform->T[1]>>xform->T[2];
	}
	cout<<"Read animation file: <"<<filename1<<">"<<endl;

	fin2>>nf>>c;
	while(!fin2.eof())
	{
		fin2>>i>>c;
		xform=&xformframes[1][(int)i];
		fin2>>xform->R[0][0]>>xform->R[1][0]>>xform->R[2][0]
			>>xform->R[0][1]>>xform->R[1][1]>>xform->R[2][1]
			>>xform->R[0][2]>>xform->R[1][2]>>xform->R[2][2];
		fin2>>xform->T[0]>>xform->T[1]>>xform->T[2];
	}
	cout<<"Read animation file: <"<<filename2<<">"<<endl;
}