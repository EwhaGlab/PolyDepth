#include "PolyDepth/MeshObject.h"
#include <gl/gl.h>
//

inline
void
VmV(PQP_REAL Vr[3], const PQP_REAL V1[3], const PQP_REAL V2[3])
{
  Vr[0] = V1[0] - V2[0];
  Vr[1] = V1[1] - V2[1];
  Vr[2] = V1[2] - V2[2];
}

inline
void
VcrossV(PQP_REAL Vr[3], const PQP_REAL V1[3], const PQP_REAL V2[3])
{
  Vr[0] = V1[1]*V2[2] - V1[2]*V2[1];
  Vr[1] = V1[2]*V2[0] - V1[0]*V2[2];
  Vr[2] = V1[0]*V2[1] - V1[1]*V2[0];
}

inline
void
Vnormalize(PQP_REAL V[3])
{
  PQP_REAL d = 1.0 / sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]);
  V[0] *= d;
  V[1] *= d;
  V[2] *= d;
}


void ComputeMassProperties (const std::vector<Coord3D>& akVertex, 
							int ntris,
							std::vector<int>& aiIndex, 
							bool bBodyCoords, 
							PQP_REAL& rfMass,
							Coord3D& rkCenter, 
							PQP_REAL rkInertia[3][3])
{
    const PQP_REAL fOneDiv6 = (PQP_REAL)(1.0/6.0);
    const PQP_REAL fOneDiv24 = (PQP_REAL)(1.0/24.0);
    const PQP_REAL fOneDiv60 = (PQP_REAL)(1.0/60.0);
    const PQP_REAL fOneDiv120 = (PQP_REAL)(1.0/120.0);

    // order:  1, x, y, z, x^2, y^2, z^2, xy, yz, zx
    PQP_REAL afIntegral[10] = { (PQP_REAL)0.0, (PQP_REAL)0.0, (PQP_REAL)0.0, (PQP_REAL)0.0,
        (PQP_REAL)0.0, (PQP_REAL)0.0, (PQP_REAL)0.0, (PQP_REAL)0.0, (PQP_REAL)0.0, (PQP_REAL)0.0 };

    const int* piIndex =  const_cast<int*>(&(*(aiIndex.begin())));
    int i;
    for (i = 0; i < ntris; i++)
    {
        // get vertices of triangle i
        Coord3D kV0 = akVertex[*piIndex++];
        Coord3D kV1 = akVertex[*piIndex++];
		Coord3D kV2 = akVertex[*piIndex++];

        // get cross product of edges
        Coord3D kV1mV0 = kV1 - kV0;
        Coord3D kV2mV0 = kV2 - kV0;
        Coord3D kN = kV1mV0%kV2mV0;

        // compute integral terms
        PQP_REAL fTmp0, fTmp1, fTmp2;
        PQP_REAL fF1x, fF2x, fF3x, fG0x, fG1x, fG2x;
        fTmp0 = kV0.X() + kV1.X();
        fF1x = fTmp0 + kV2.X();
        fTmp1 = kV0.X()*kV0.X();
        fTmp2 = fTmp1 + kV1.X()*fTmp0;
        fF2x = fTmp2 + kV2.X()*fF1x;
        fF3x = kV0.X()*fTmp1 + kV1.X()*fTmp2 + kV2.X()*fF2x;
        fG0x = fF2x + kV0.X()*(fF1x + kV0.X());
        fG1x = fF2x + kV1.X()*(fF1x + kV1.X());
        fG2x = fF2x + kV2.X()*(fF1x + kV2.X());

        PQP_REAL fF1y, fF2y, fF3y, fG0y, fG1y, fG2y;
        fTmp0 = kV0.Y() + kV1.Y();
        fF1y = fTmp0 + kV2.Y();
        fTmp1 = kV0.Y()*kV0.Y();
        fTmp2 = fTmp1 + kV1.Y()*fTmp0;
        fF2y = fTmp2 + kV2.Y()*fF1y;
        fF3y = kV0.Y()*fTmp1 + kV1.Y()*fTmp2 + kV2.Y()*fF2y;
        fG0y = fF2y + kV0.Y()*(fF1y + kV0.Y());
        fG1y = fF2y + kV1.Y()*(fF1y + kV1.Y());
        fG2y = fF2y + kV2.Y()*(fF1y + kV2.Y());

        PQP_REAL fF1z, fF2z, fF3z, fG0z, fG1z, fG2z;
        fTmp0 = kV0.Z() + kV1.Z();
        fF1z = fTmp0 + kV2.Z();
        fTmp1 = kV0.Z()*kV0.Z();
        fTmp2 = fTmp1 + kV1.Z()*fTmp0;
        fF2z = fTmp2 + kV2.Z()*fF1z;
        fF3z = kV0.Z()*fTmp1 + kV1.Z()*fTmp2 + kV2.Z()*fF2z;
        fG0z = fF2z + kV0.Z()*(fF1z + kV0.Z());
        fG1z = fF2z + kV1.Z()*(fF1z + kV1.Z());
        fG2z = fF2z + kV2.Z()*(fF1z + kV2.Z());

        // update integrals
        afIntegral[0] += kN.X()*fF1x;
        afIntegral[1] += kN.X()*fF2x;
        afIntegral[2] += kN.Y()*fF2y;
        afIntegral[3] += kN.Z()*fF2z;
        afIntegral[4] += kN.X()*fF3x;
        afIntegral[5] += kN.Y()*fF3y;
        afIntegral[6] += kN.Z()*fF3z;
        afIntegral[7] += kN.X()*(kV0.Y()*fG0x + kV1.Y()*fG1x + kV2.Y()*fG2x);
        afIntegral[8] += kN.Y()*(kV0.Z()*fG0y + kV1.Z()*fG1y + kV2.Z()*fG2y);
        afIntegral[9] += kN.Z()*(kV0.X()*fG0z + kV1.X()*fG1z + kV2.X()*fG2z);
    }

    afIntegral[0] *= fOneDiv6;
    afIntegral[1] *= fOneDiv24;
    afIntegral[2] *= fOneDiv24;
    afIntegral[3] *= fOneDiv24;
    afIntegral[4] *= fOneDiv60;
    afIntegral[5] *= fOneDiv60;
    afIntegral[6] *= fOneDiv60;
    afIntegral[7] *= fOneDiv120;
    afIntegral[8] *= fOneDiv120;
    afIntegral[9] *= fOneDiv120;

    // mass
    rfMass = afIntegral[0];

    // center of mass
    rkCenter =
        Coord3D(afIntegral[1],afIntegral[2],afIntegral[3])/rfMass;

    // inertia relative to world origin
    rkInertia[0][0] = afIntegral[5] + afIntegral[6];
    rkInertia[0][1] = -afIntegral[7];
    rkInertia[0][2] = -afIntegral[9];
    rkInertia[1][0] = rkInertia[0][1];
    rkInertia[1][1] = afIntegral[4] + afIntegral[6];
    rkInertia[1][2] = -afIntegral[8];
    rkInertia[2][0] = rkInertia[0][2];
    rkInertia[2][1] = rkInertia[1][2];
    rkInertia[2][2] = afIntegral[4] + afIntegral[5];

    // inertia relative to center of mass
    if ( bBodyCoords )
    {
        rkInertia[0][0] -= rfMass*(rkCenter.Y()*rkCenter.Y() +
            rkCenter.Z()*rkCenter.Z());
        rkInertia[0][1] += rfMass*rkCenter.X()*rkCenter.Y();
        rkInertia[0][2] += rfMass*rkCenter.Z()*rkCenter.X();
        rkInertia[1][0] = rkInertia[0][1];
        rkInertia[1][1] -= rfMass*(rkCenter.Z()*rkCenter.Z() +
            rkCenter.X()*rkCenter.X());
        rkInertia[1][2] += rfMass*rkCenter.Y()*rkCenter.Z();
        rkInertia[2][0] = rkInertia[0][2];
        rkInertia[2][1] = rkInertia[1][2];
        rkInertia[2][2] -= rfMass*(rkCenter.X()*rkCenter.X() +
            rkCenter.Y()*rkCenter.Y());
    }
}


void MeshObject::ReadTriFile(const char *tris_file)
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
  PQP_REAL a,b,c;
  fscanf(fp,"%s",str);
  fscanf(fp,"%d",&nvertics);
  fscanf(fp,"%d",&ntris);
   tri = new ModelTriNew[ntris];
  
  //PQP_REAL *p=new PQP_REAL[3*nvertics];
  for (i = 0; i < nvertics; i++)
  {
	  fscanf(fp,"%lf %lf %lf",&a,&b,&c);
	  //p[3*i+0] = a;
	  //p[3*i+1] = b;
	  //p[3*i+2] = c;
	  
	  
  }
  
  // ntris: numbers of triangles 
  //P1 the one vertex, P2 P3
  for (i = 0; i < ntris; i++)
  {
	  fscanf(fp,"%d %d %d", &i1,&i2,&i3);
	  tri[i].p0[0] = vertices_[i1][0]; //p[i1*3];
	  tri[i].p0[1] = vertices_[i1][1]; // p[i1*3+1];
	  tri[i].p0[2] = vertices_[i1][2]; // p[i1*3+2];
	  
	  tri[i].p1[0] = vertices_[i2][0]; // p[i2*3];
	  tri[i].p1[1] = vertices_[i2][1]; // p[i2*3+1];
	  tri[i].p1[2] = vertices_[i2][2]; // p[i2*3+2];
	  
	  tri[i].p2[0] = vertices_[i3][0]; // p[i3*3];
	  tri[i].p2[1] = vertices_[i3][1]; // p[i3*3+1];
	  tri[i].p2[2] = vertices_[i3][2]; // p[i3*3+2];
	  PQP_REAL a[3],b[3];
      VmV(a,tri[i].p1,tri[i].p0);
      VmV(b,tri[i].p2,tri[i].p0);
      VcrossV(tri[i].n,a,b);
      Vnormalize(tri[i].n);
  }
  
  
  fclose(fp);
  //delete []p;

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

void MeshObject::ReadOffFile(const char *off_file)
{
  FILE *fp = fopen(off_file,"r");
  if (fp == NULL)
  { 
    fprintf(stderr,"Model Constructor: Couldn't open %s\n",off_file); 
    exit(-1); 
  }

//  fscanf(fp,"%d",&ntris);
 
  char str[10];
  int nvertics,ntris;
  int i,i1,i2,i3;
  PQP_REAL a,b,c;
  fscanf(fp,"%s",str);
  fscanf(fp,"%d",&nvertics);
  fscanf(fp,"%d",&ntris);
  fscanf(fp,"%d", &i); // dummy
   tri = new ModelTriNew[ntris];
  
  //PQP_REAL *p=new PQP_REAL[3*nvertics];
  for (i = 0; i < nvertics; i++)
  {
	  fscanf(fp,"%lf %lf %lf",&a,&b,&c);
	  //p[3*i+0] = a;
	  //p[3*i+1] = b;
	  //p[3*i+2] = c;
	  
	  
  }
  
  // ntris: numbers of triangles 
  //P1 the one vertex, P2 P3
  for (i = 0; i < ntris; i++)
  {
	  int temp;

	  fscanf(fp,"%d %d %d %d", &temp, &i1,&i2,&i3);


	  tri[i].p0[0] = vertices_[i1][0]; //p[i1*3];
	  tri[i].p0[1] = vertices_[i1][1]; // p[i1*3+1];
	  tri[i].p0[2] = vertices_[i1][2]; // p[i1*3+2];
	  
	  tri[i].p1[0] = vertices_[i2][0]; // p[i2*3];
	  tri[i].p1[1] = vertices_[i2][1]; // p[i2*3+1];
	  tri[i].p1[2] = vertices_[i2][2]; // p[i2*3+2];
	  
	  tri[i].p2[0] = vertices_[i3][0]; // p[i3*3];
	  tri[i].p2[1] = vertices_[i3][1]; // p[i3*3+1];
	  tri[i].p2[2] = vertices_[i3][2]; // p[i3*3+2];

	  PQP_REAL a[3],b[3];
      VmV(a,tri[i].p1,tri[i].p0);
      VmV(b,tri[i].p2,tri[i].p0);
      VcrossV(tri[i].n,a,b);
      Vnormalize(tri[i].n);
  }
  
  
  fclose(fp);
  //delete []p;

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

MeshObject::MeshObject(const MeshObject& cp)
: c2a_model_(cp.c2a_model_), 
  vertices_(cp.vertices_),
  normals_(cp.normals_),
  triangles_(cp.triangles_),
  clear_conf_(cp.clear_conf_),
  tri(cp.tri),
  minimum_(cp.minimum_),
  maximum_(cp.maximum_),
  model_center_(cp.model_center_),
  radius_(cp.radius_),
  display_list(cp.display_list),
  center_of_mass_(cp.center_of_mass_),
  mass_(cp.mass_),
  last_triangle(cp.last_triangle)
{
	for (int i = 0 ; i < 3 ; i++)
		for (int j = 0 ; j < 3 ; j++)
			inertia_[i][j] = cp.inertia_[i][j];
}

MeshObject::MeshObject(const char *modelFile1)
: clear_conf_(), vertices_(), triangles_(), normals_()
{
	FILE *fp;
	int ntris, nvertics;
	PQP_REAL a, b, c;
	char str[100];
	PQP_REAL p1[3],p2[3],p3[3];

	int i1,i2,i3;

	// model 1

	strcpy(name_, modelFile1);
	printf("reading the file: %s\n",modelFile1);
	int len = strlen(modelFile1);
	bool tri_model1 = true;

	if (!strcmp(&modelFile1[len-3],"OFF") || !strcmp(&modelFile1[len-3],"off"))
		tri_model1 = false;

	c2a_model_ = new C2A_Model();
	int tmp;

	fp = fopen(modelFile1,"r");
	if (!fp) {
		printf("file %s does not exist\n", modelFile1);
		exit(0);
	}

	fscanf(fp,"%s",str);
	fscanf(fp,"%d",&nvertics);
	fscanf(fp,"%d",&ntris);
	if (!tri_model1)
		fscanf(fp,"%d",&tmp);

	printf("building the C2A_BVH for the model: %s\n",modelFile1);

	vertices_.resize(nvertics);
	model_center_ = Coord3D(0,0,0);

	for (int i = 0; i < nvertics; i++)
	{
		fscanf(fp,"%lf %lf %lf",&a,&b,&c);

		Coord3D aa((PQP_REAL)a, (PQP_REAL)b, (PQP_REAL)c);
		model_center_ += aa;
		vertices_[i].Set_Value((PQP_REAL)a, (PQP_REAL)b, (PQP_REAL)c );

	}

	model_center_ /= (PQP_REAL)nvertics;

	// ntris: numbers of triangles 
	//P1 the one vertex, P2 P3
	for (int i = 0; i < ntris; i++)
	{
		int tmp;
		if (!tri_model1) {
			fscanf(fp,"%d %d %d %d",&tmp, &i1,&i2,&i3);
		}
		else {
			fscanf(fp,"%d %d %d",&i1,&i2,&i3);
		}
		triangles_.push_back(i1);
		triangles_.push_back(i2);
		triangles_.push_back(i3);
	}

	fclose(fp);

	ComputeMassProperties(vertices_, ntris, triangles_, false, mass_, center_of_mass_, inertia_);
	printf("center of mass of this object %f %f %f\n", center_of_mass_.X(), center_of_mass_.Y(), center_of_mass_.Z());
	printf("inertia\n %f %f %f\n %f %f %f\n %f %f %f\n", inertia_[0][0], inertia_[0][1], inertia_[0][2], inertia_[1][0], inertia_[1][1], inertia_[1][2],inertia_[2][0], inertia_[2][1], inertia_[2][2]);

	for (int i = 0 ; i < vertices_.size() ; i++) {
		vertices_[i] = vertices_[i] - center_of_mass_;
	}
	// read them for rendering
	if (tri_model1) {
		ReadTriFile(modelFile1);
	}
	else {
		ReadOffFile(modelFile1);
	}



	c2a_model_->BeginModel();

	int count = 0;
	for (int i = 0 ; i < ntris ; i++) {
		i1 = triangles_[count]; count++;
		i2 = triangles_[count]; count++;
		i3 = triangles_[count]; count++;

		p1[0] = vertices_[i1][0]; //p[i1*3];
		p1[1] = vertices_[i1][1]; // p[i1*3+1];
		p1[2] = vertices_[i1][2]; // p[i1*3+2];

		p2[0] = vertices_[i2][0]; // p[i2*3];
		p2[1] = vertices_[i2][1]; // p[i2*3+1];
		p2[2] = vertices_[i2][2]; // p[i2*3+2];

		p3[0] = vertices_[i3][0]; // p[i3*3];
		p3[1] = vertices_[i3][1]; // p[i3*3+1];
		p3[2] = vertices_[i3][2]; // p[i3*3+2];

		c2a_model_->AddTri(p1,p2,p3,i,i1,i2,i3);
	}
	c2a_model_->EndModel(); 

	last_triangle = c2a_model_->last_tri;

	//ComputeMassProperties(vertices_, ntris, triangles_, false, mass_, center_of_mass_, inertia_);

#ifndef _MAXDOUBLE
#define _MAXDOUBLE	((double)1.7976931348623158e+308)
#endif

#define _MINFLOAT	((float)1.17549435e-38)
#define _MINDOUBLE	((double)2.2250738585072014e-308)

    minimum_ = Coord3D(_MAXDOUBLE, _MAXDOUBLE, _MAXDOUBLE);
    maximum_ = Coord3D(-_MAXDOUBLE, -_MAXDOUBLE, -_MAXDOUBLE);

	radius_ = 0.0;
	for (int i = 0 ; i < vertices_.size() ; i++) {
		Coord3D pt = vertices_[i];
		PQP_REAL len = pt.Length();
		
		if (len > radius_) radius_ = len;

		if (pt[0] < minimum_[0]) minimum_[0] = pt[0];
		if (pt[1] < minimum_[1]) minimum_[1] = pt[1];
		if (pt[2] < minimum_[2]) minimum_[2] = pt[2];
		if (pt[0] > maximum_[0]) maximum_[0] = pt[0];
		if (pt[1] > maximum_[1]) maximum_[1] = pt[1];
		if (pt[2] > maximum_[2]) maximum_[2] = pt[2];
	}

	printf("bounding %f %f %f, %f %f %f\n", minimum_[0], minimum_[1], minimum_[2], maximum_[0], maximum_[1], maximum_[2]);
}




MeshObject::~MeshObject()
{
  delete [] tri;
}

void
MeshObject::Draw()
{
  glCallList(display_list);
}
