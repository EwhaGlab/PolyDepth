# PolyDepth

PolyDepth is an interactive penetration depth calculation library written in C++ for rigid polygon soup models. 
For two intersected polygonal models, PolyDepth can compute a minimum distance to separate them by translation:
* It can compute a single distance value to separate the intersected model
* It can also report a set of distance values for each intersected _region_
* It has different options to choose for initialization 

The code is written using Visual C++ 9.0 in MS Windows and is not tested on other platforms yet.
To run the code, VC++ 9.0 or higher is needed.

## Dependencies

To run PolyDepth, two libraries are needed, **PQP** and **C2A**.

1. Please download PQP at [here](http://www.cs.unc.edu/%7Egeom/SSV/index.html). **PQP_v1.3** version is needed.


2. Copy the entire PQP_v1.3 folder into the root path.


3. Please download C2A at [here](http://code.google.com/p/c2a-ewha/). **C2A_1.3** or higher version is needed.


4. Copy only the C2A folder into the root path.


5. Copy the files PQP.sln and PQP.vcproj into PQP_v1.3 folder.


6. Comment the follow sentences in `PQP_Compile.h`
    ```c++
    inline float sqrt(float x) { return (float)sqrt((double)x); }
    inline float cos(float x) { return (float)cos((double)x); }
    inline float sin(float x) { return (float)sin((double)x); }
    inline float fabs(float x) { return (float)fabs((double)x); }
    ```

## About the project
In PolyDepth.sln, two projects exist other than C2A and PQP:

* **PolyDepthDemo**: the Demo function for showing how to use PolyDepth.lib

* **PolyDepth**: interactive translational penetration depth calculation, and form PolyDepth.lib

The main function:
```c++
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
```

The result of PolyDepth is saved in a result folder under the root path.

## Demo

To run the demo, you need [GLUT](https://www.opengl.org/resources/libraries/glut/) & [OpenGL](https://www.opengl.org/). Please make sure the **Startup project** is setup correctly to **PolyDepthDemo**. 


## Citation
PolyDepth was presented in **SIGGRAPH 2012**. Please visit our [website](http://graphics.ewha.ac.kr/polydepth/) for more deatils.
To cite PolyDepth in your academic research, please use the following bibtex lines:
```bib
@article{10.1145/2077341.2077346,
author = {Je, Changsoo and Tang, Min and Lee, Youngeun and Lee, Minkyoung and Kim, Young J.},
title = {PolyDepth: Real-Time Penetration Depth Computation Using Iterative Contact-Space Projection},
year = {2012},
issue_date = {January 2012},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
volume = {31},
number = {1},
issn = {0730-0301},
url = {https://doi.org/10.1145/2077341.2077346},
doi = {10.1145/2077341.2077346},
abstract = {We present a real-time algorithm that finds the Penetration Depth (PD) between general polygonal models based on iterative and local optimization techniques. Given an in-collision configuration of an object in configuration space, we find an initial collision-free configuration using several methods such as centroid difference, maximally clear configuration, motion coherence, random configuration, and sampling-based search. We project this configuration on to a local contact space using a variant of continuous collision detection algorithm and construct a linear convex cone around the projected configuration. We then formulate a new projection of the in-collision configuration onto the convex cone as a Linear Complementarity Problem (LCP), which we solve using a type of Gauss-Seidel iterative algorithm. We repeat this procedure until a locally optimal PD is obtained. Our algorithm can process complicated models consisting of tens of thousands triangles at interactive rates.},
journal = {ACM Trans. Graph.},
month = {feb},
articleno = {5},
numpages = {14},
keywords = {collision detection, polygon-soups, penetration depth, Animation, dynamics}
}
```


## Contact

If you have any questions or find some bugs, please feel free to connect with **Min Tang** by e-mail: tangmin@ewha.ac.kr or tangminewha@gmail.com.
