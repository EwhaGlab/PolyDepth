#ifndef __MeshObject_h
#define __MeshObject_h

#include <C2A/LinearMath.h>
#include <C2A/C2A.h>
#include <vector>
#include "PolyDepth/PolyDepth.h"


struct ModelTriNew
{
  double p0[3], p1[3], p2[3];
  double n[3];
};

class MeshObject {

public:
	friend class Controller;

	MeshObject(const char *file_name);


	MeshObject(const MeshObject& cp);
	MeshObject() {}
	
	virtual ~MeshObject();
	void Draw();

	
	std::vector<Coord3D>& vertices() { return vertices_; }
	C2A_Model* c2a_model() { return c2a_model_; }

	Tri*       LastTriangle() { return last_triangle; }
	ClearConfigurations& clear_conf() { return clear_conf_; }  


	Coord3D minimum() { return minimum_; }
	Coord3D maximum() { return maximum_; }
	PQP_REAL radius() { return radius_; }
	PQP_REAL mass() const {return mass_; }


	PQP_REAL				inertia_[3][3];

private:

	C2A_Model*				c2a_model_;
	std::vector<Coord3D>	vertices_;
	std::vector<Coord3D>	normals_;
	std::vector<int>		triangles_;

	ClearConfigurations     clear_conf_;

	ModelTriNew				*tri;
	Tri                     *last_triangle;  
public:
	char					name_[100];
private:
	// bounding box
	Coord3D					minimum_;
	Coord3D					maximum_;

	Coord3D					model_center_;

	// bounding sphere
	PQP_REAL				radius_;

	int						display_list;

	PQP_REAL				mass_;
	
	Coord3D					center_of_mass_;

	void ReadTriFile(const char *tris_file);
	void ReadOffFile(const char *off_file);
};

#endif