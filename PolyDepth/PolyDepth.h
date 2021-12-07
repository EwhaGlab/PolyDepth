/*************************************************************************\

  Copyright 2009	Computer Graphics Lab, 
					Ewha Womans University,
					Seoul, Korea.
  All Rights Reserved.

  The authors may be contacted via:

  Mailing Address:     Xinyu Zhang
                       Min Tang
                       Department of Computer Science & Engineering
                       Ewha Womans University
                       Seoul, 120-750, Korea.

  EMail:               zhangxy@ewha.ac.kr
                       tangmin@ewha.ac.kr

\**************************************************************************/

#ifndef __PolyDepth_h
#define __PolyDepth_h

#include <stdio.h>
#include <stdlib.h>

#include <vector>

#include <C2A/C2A.h>
#include <C2A/InterpMotion.h>
#include <C2A/LinearMath.h>
#include "PolyDepth/ClearConfiguration.h"

enum PolyDepthResult {
	kNoPenetration,
	kPenetration,
	kFailed
};


struct PolyDepthReturnValue {
	PolyDepthResult result_;
	PQP_REAL		distance_;
};


/*!
	Main PolyDepth Algorithm
*/
PolyDepthReturnValue 
PolyDepth(Transform*				object_1_pose,				// input pose for the object_1
		  C2A_Model*				object_1,					// input model of object_1
		  Transform*				object_2_pose,				// input pose for the object_2
		  C2A_Model *				object_2,					// input model of object_2
		  Tri*                      last_triangle1,
		  Tri*                      last_triangle2,
		  const std::vector<Coord3D>& object_1_vertices,		// vertices of object_1 
		  const std::vector<Coord3D>& object_2_vertices,		// vertices of object_2
		  Transform&				output_object_1_pose,		// output: final pose of object_1 when it is seperated from the collided object_2
		  Transform&				output_object_2_pose,		// output: no use
		  std::vector<Coord3D>&		local_penetration_depth,	// output: local penetration depths
		  std::vector<Coord3D>&		local_penetration_features1,	// output: local penetration features from object_1
		  std::vector<Coord3D>&		local_penetration_features2,	// output: local penetration features from object_2
		  Coord3D&					global_penetration_depth,	// output: global penetration depth
		  int&                      number_of_iteration,

		  // clear configuration could be following: 
		  //  1. maximally clear configuration computed via FindMaximallyClearConfigure
		  //  2. previous object1's pose (translation and rotation) that may be free of contact 
		  //  3. any pose that user thinks free of contact, in this case use the constructor
		  //     ClearConfiguration(Coord3D).
		  const						ClearConfigurations& clear_conf,
		  bool						use_motion_coherance = false,
		  bool						use_max_clear_conf = false,

		  // NOTE: call the object distance "d". if 0.0 < d < absolute_contact_configuration, 
		  // we claim that two objects are in contact configuration with each other. So, this number 
		  // is ultimately important for many applications. You can simply give a minus value,
		  // that Polydepth can pick a proper number for you. In case of minus value 
		  // so Polydepth computes this value to its purpose for you,
		  // i.e. 0.0012 * bbox size of object 2. In those simulation environments where
		  // simulation precision is important, you may specify this value by yourself.
		  //
		  PQP_REAL					absolute_contact_configuration = -1.0,
		  // provide grid resolution of the feature space; smaller value yields smaller number of features
		  PQP_REAL					feature_clustering_resolution = 5); 

// when Polydepth returns kNoPentration, call this function to check if two objects are within the distance
// of absolute_contact_configuration. If so, local penetration depth info are returned.
PolyDepthReturnValue 
SkinDepth(Transform*				object_1_pose,				// input pose for the object_1
		  C2A_Model*				object_1,					// input model of object_1
		  Transform*				object_2_pose,				// input pose for the object_2
		  C2A_Model *				object_2,					// input model of object_2
		  const std::vector<Coord3D>& object_1_vertices,		// vertices of object_1 
		  const std::vector<Coord3D>& object_2_vertices,		// vertices of object_2
		  Transform&				output_object_1_pose,		// output: final pose of object_1 when it is seperated from the collided object_2
		  Transform&				output_object_2_pose,		// output: no use
		  std::vector<Coord3D>&		local_penetration_depth,	// output: local penetration depths
		  std::vector<Coord3D>&		local_penetration_features1,	// output: local penetration features from object_1
		  std::vector<Coord3D>&		local_penetration_features2,	// output: local penetration features from object_2
		  Coord3D&					global_penetration_depth,	// output: global penetration depth

		  // NOTE: call the object distance "d". if 0.0 < d < absolute_contact_configuration, 
		  // we claim that two objects are in contact configuration with each other. So, this number 
		  // is ultimately important for many applications. You can simply give a minus value,
		  // that Polydepth can pick a proper number for you. In case of minus value 
		  // so Polydepth computes this value to its purpose for you,
		  // i.e. 0.0012 * bbox size of object 2. In those simulation environments where
		  // simulation precision is important, you may specify this value by yourself.
		  //
		  PQP_REAL					absolute_contact_configuration = -1.0,
		  // provide grid resolution of the feature space; smaller value yields smaller number of features
		  PQP_REAL					feature_clustering_resolution = 5); 
/*!
	Find maximally clear configurations

	\param object_vertices: vertices of input object
	\param object: target object about which this function creates the MCC(Maximally clear configuration)
	\param maximally_clear_conf: maximally clear vertices output
	\param solid: indicates whether if solid, we check inside/outside and exclude the inside points
	\param grid_resolution: to form grid to compute the maximally clear conf, we need grid resolution,
							the number of grid points inside the object bounding volume.

*/	
bool FindMaximallyClearConfigure(const std::vector<Coord3D>& object_vertices, 
								 C2A_Model* object, 

								 //ClearConfigurations& maximally_clear_conf,
								 bool solid = true,			
								 int grid_resolution = 1000);



#endif