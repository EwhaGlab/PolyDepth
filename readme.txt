PolyDepth is an interactive penetration depth calculation library written in C++ for rigid polygon soup models. The code is written using Visual C++ 9.0 in MS Windows and is not tested on other platforms yet.
To run the code, VC++ 9.0 is needed. 
/*****************************************************************************/
To run PolyDepth, two libraries are needed, PQP and C2A.


Please download:
C2A at http://code.google.com/p/c2a-ewha/    
*C2A_1.3 or higher version is needed.
*Copy C2A folder into the root path of PolyDepth_1.0.

/*****************************************************************************/
The files of PQP are also needed. 

    Please download:
    PQP at http://www.cs.unc.edu/%7Egeom/SSV/index.html
    

1.Copy the PQP_v1.3 folder into root path of PolyDepth_1.0.

2.Copy the files PQP.sln and PQP.vcproj into PQP_v1.3 folder.

3.Comment the follow sentences in PQP_Compile.h
inline float sqrt(float x) { return (float)sqrt((double)x); }

inline float cos(float x) { return (float)cos((double)x); }

inline float sin(float x) { return (float)sin((double)x); }

inline float fabs(float x) { return (float)fabs((double)x); }
/*****************************************************************************/

In PolyDepth.sln, four projects exist:

PolyDepthDemo: the Demo function for showing how to use PolyDepth.lib.

PolyDepth: interactive translational penetration depth calculation, and form PolyDepth.lib
The main function:

PolyDepthReturnValue 
PolyDepth(Transform*				object_1_pose,			// input pose for the object_1
          C2A_Model*				object_1,			// input model of object_1
	  Transform*				object_2_pose,			// input pose for the object_2
	  C2A_Model *				object_2,			// input model of object_2
	  Tri*                                  last_triangle1,
	  Tri*                                  last_triangle2,
	  const std::vector<Coord3D>&           object_1_vertices,		// vertices of object_1 
	  const std::vector<Coord3D>&           object_2_vertices,		// vertices of object_2
	  Transform&				output_object_1_pose,		// output: final pose of object_1 when it is seperated from the collided object_2
	  Transform&				output_object_2_pose,		// output: no use
	  std::vector<Coord3D>&		        local_penetration_depth,	// output: local penetration depths
	  std::vector<Coord3D>&		        local_penetration_features1,	// output: local penetration features from object_1
	  std::vector<Coord3D>&		        local_penetration_features2,	// output: local penetration features from object_2
          Coord3D&			        global_penetration_depth,	// output: global penetration depth
          int&                                  number_of_iteration,

		  // clear configuration could be following: 
		  //  1. maximally clear configuration computed via FindMaximallyClearConfigure
		  //  2. previous object1's pose (translation and rotation) that may be free of contact 
		  //  3. any pose that user thinks free of contact, in this case use the constructor
		  //     ClearConfiguration(Coord3D).
	   const				ClearConfigurations& clear_conf,
	   bool				        use_motion_coherance = false,
	   bool				        use_max_clear_conf = false,

		  // NOTE: call the object distance "d". if 0.0 < d < absolute_contact_configuration, 
		  // we claim that two objects are in contact configuration with each other. So, this number 
		  // is ultimately important for many applications. You can simply give a minus value,
		  // that Polydepth can pick a proper number for you. In case of minus value 
		  // so Polydepth computes this value to its purpose for you,
		  // i.e. 0.0012 * bbox size of object 2. In those simulation environments where
		  // simulation precision is important, you may specify this value by yourself.
		  //
	    PQP_REAL				absolute_contact_configuration = -1.0,
		  // provide grid resolution of the feature space; smaller value yields smaller number of features
	    PQP_REAL				feature_clustering_resolution = 5);


The result of PolyDepth is saved in result folder under root path.



C2A: continuous collision detection based on PQP soft package, and form C2A.lib




/*****************************************************************************/

If you have any questions or find some bugs, please feel free to connect with Min Tang by EMail: tangmin@ewha.ac.kr or tangminewha@gmail.com.
