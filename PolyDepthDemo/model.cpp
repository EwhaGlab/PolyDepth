/*************************************************************************\

  Copyright 1999 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             E. Larsen
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:               geom@cs.unc.edu


\**************************************************************************/
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "GL/glut.h"
#include "model.h"



Model::Model(const char *tris_file)
{
  FILE *fp = fopen(tris_file,"r");
  if (fp == NULL)
  { 
    fprintf(stderr,"Model Constructor: Couldn't open %s\n",tris_file); 
    exit(-1); 
  }

//  fscanf(fp,"%d",&ntris);
 
  char str[10];
  int nvertics,ntris;
  int i,i1,i2,i3;
  double a,b,c;
  fscanf(fp,"%s",str);
  fscanf(fp,"%d",&nvertics);
  fscanf(fp,"%d",&ntris);
   tri = new ModelTri[ntris];
  
  double *p=new double[3*nvertics];
  for (i = 0; i < nvertics; i++)
  {
	  fscanf(fp,"%lf %lf %lf",&a,&b,&c);
	  p[3*i+0] = a;
	  p[3*i+1] = b;
	  p[3*i+2] = c;
	  
	  
  }
  
  // ntris: numbers of triangles 
  //P1 the one vertex, P2 P3
  for (i = 0; i < ntris; i++)
  {
	  fscanf(fp,"%d %d %d",&i1,&i2,&i3);
	  tri[i].p0[0] = p[i1*3];
	  tri[i].p0[1] = p[i1*3+1];
	  tri[i].p0[2] = p[i1*3+2];
	  
	  tri[i].p1[0] = p[i2*3];
	  tri[i].p1[1] = p[i2*3+1];
	  tri[i].p1[2] = p[i2*3+2];
	  
	  tri[i].p2[0] = p[i3*3];
	  tri[i].p2[1] = p[i3*3+1];
	  tri[i].p2[2] = p[i3*3+2];
	  double a[3],b[3];
      VmV(a,tri[i].p1,tri[i].p0);
      VmV(b,tri[i].p2,tri[i].p0);
      VcrossV(tri[i].n,a,b);
      Vnormalize(tri[i].n);
	  
	  
  }
  
  
  fclose(fp);
  delete []p;

  // generate display list

  display_list = glGenLists(1);
  glNewList(display_list,GL_COMPILE);
  glBegin(GL_TRIANGLES);
  for (i = 0; i < ntris; i++)
  {
    glNormal3dv(tri[i].n);
    glVertex3dv(tri[i].p0);
    glVertex3dv(tri[i].p1);
    glVertex3dv(tri[i].p2);
  }
  glEnd();
  glEndList();  
}

Model::~Model()
{
  delete [] tri;
}

void
Model::Draw()
{
  glCallList(display_list);
}

void
Model::DrawTri(int index)
{
  glBegin(GL_TRIANGLES);
  glVertex3dv(tri[index].p0);
  glVertex3dv(tri[index].p1);
  glVertex3dv(tri[index].p2);
  glEnd();
}


void 
Model::CenterOfMass()
{
    int i;
    double area_x2;
    double total_area;

    com.Zero();//set the coordinate of center to be zero
    total_area = 0.0;
    for( i = 0; i < ntris; i++ ) {
			  double E0[3], E1[3], S[3];
				VmV(E0, tri[i].p0, tri[i].p1);// E0 is a vector point from P1 to P0
				VmV(E1, tri[i].p0, tri[i].p2);
				VcrossV(S, E0, E1);
				area_x2=sqrt(S[0]*S[0] + S[1]*S[1] + S[2]*S[2]);//area_x2 is area of triangle P1P0P2
        total_area += area_x2;
        com += Coord3D(	tri[i].p0[0] + tri[i].p1[0] +tri[i].p2[0],
												tri[i].p0[1] + tri[i].p1[1] +tri[i].p2[1],
												tri[i].p0[2] + tri[i].p1[2] +tri[i].p2[2]) * area_x2;
    }
    com /= 3.0 * total_area;
}

void 
Model::Radius ()
{
/*
        int i;
    		radius=0.0;
    		double d;
        for( i = 0; i < ntris; i++ ) {
    				d=(Coord3D(tri[i].p0[0], tri[i].p0[1], tri[i].p0[2])-com).abs();
    				if( d > radius ) radius = d;
    
    				d=(Coord3D(tri[i].p1[0], tri[i].p1[1], tri[i].p1[2])-com).abs();
    				if( d > radius ) radius = d;
    
    				d=(Coord3D(tri[i].p2[0], tri[i].p2[1], tri[i].p2[2])-com).abs();
    				if( d > radius ) radius = d;
    
        }
        radius=sqrt(radius);*/
    
}