#include "PolyDepth/PolyDepth.h"
#include "PolyDepthDemo/stopwatch.h"
#include <map>

//#define LOCAL_PENETRATIION


using namespace std;

std::vector<Coord3D> kMaxClearConf;


inline void Transform2PQP(Transform *t, PQP_REAL R[3][3], PQP_REAL T[3])
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			R[i][j] = t->Rotation()[i][j];
		}
		T[i] = t->Translation()[i];
	}
}

void
GuessContactFreeSpace(Transform*		object_1_pose,
					  C2A_Model*		object_1,
					  Transform*		object_2_pose,
					  C2A_Model*		object_2,
					  const				ClearConfigurations& clear_conf,
					  bool				use_motion_coherance,
					  bool				use_max_clear_conf,
					  Transform&		output_space)
{
	
	PQP_CollideResult collide_result;

	PQP_REAL max_bbox_size = sqrt(object_2->b->d[0] * object_2->b->d[0] + 
								  object_2->b->d[1] * object_2->b->d[1] + 
								  object_2->b->d[2] * object_2->b->d[2]);
	PQP_REAL max_bbox_size1 = sqrt(object_1->b->d[0] * object_1->b->d[0] + 
								  object_1->b->d[1] * object_1->b->d[1] + 
								  object_1->b->d[2] * object_1->b->d[2]);

	PQP_REAL obj2_rot[3][3], obj1_rot[3][3];
	PQP_REAL obj2_tran[3], obj1_tran[3];

	Transform2PQP(object_2_pose, obj2_rot, obj2_tran);

	
	bool found = false;
	bool direction_found = false;
	Coord3D dir; 



	if (use_motion_coherance && clear_conf.size()) {

		std::vector<bool> selected(clear_conf.size());

		for (int i = 0 ; i < selected.size() ; i++)
			selected[i] = false;

		while(true) {

			PQP_REAL min_dist = 1.0e+15;
			int process_id = -1;

			for (int i = 0 ; i < clear_conf.size() ; i++) {
				
				if (selected[i]) continue;
				if (!clear_conf[i].used()) {
					continue;
				}

				if (clear_conf[i].HasValidDistance()) {

					if (min_dist > clear_conf[i].distance_to_object()) {
						min_dist = clear_conf[i].distance_to_object();
						process_id = i;
					}				
				} else {
					Coord3D dis = object_1_pose->Translation() - (clear_conf[i].position() + object_2_pose->Translation());
					PQP_REAL len = dis.Length();
					if (min_dist > len) {
						min_dist = len;
						process_id = i;
						
					}
				}
			}
			if (process_id == -1) break;

			selected[process_id] = true;


			Coord3D p1 = object_1_pose->Translation();
			Coord3D p2 = clear_conf[process_id].position()+object_2_pose->Translation();
			if (p2.Dist(p1) < 1.0e-15) continue;

			direction_found = true;
			dir = p2-p1;
			double tran_amount = p1.Dist(p2);
			dir.Normalize();

			output_space.Set_Rotation(object_1_pose->Rotation());
			output_space.Set_Translation(p2);

			Transform2PQP(&output_space, obj1_rot, obj1_tran);

			double rel_err = 0.0, abs_err = 0.0;

			C2A_Collide(&collide_result, obj1_rot, obj1_tran, object_1,
				obj2_rot, obj2_tran, object_2, C2A_FIRST_CONTACT );

			if (!collide_result.Colliding()) {

				found = true;
				break;
			}
			else {
				//printf("slight push dir %f %f %f\n", dir.X(), dir.Y(), dir.Z());
				output_space.Set_Translation(p2 + (dir * 1.0e-3));
				Transform2PQP(&output_space, obj1_rot, obj1_tran);
				C2A_Collide(&collide_result, obj1_rot, obj1_tran, object_1,
							obj2_rot, obj2_tran, object_2, C2A_FIRST_CONTACT );
				if (!collide_result.Colliding()) {
					found = true;
					break;
				}
				double step = (max_bbox_size1)/10.0;
				for (int i = 1 ; i < 10; i++) {
					output_space.Set_Translation(p2 + (dir * (step * i)));
					Transform2PQP(&output_space, obj1_rot, obj1_tran);
					C2A_Collide(&collide_result, obj1_rot, obj1_tran, object_1,
								obj2_rot, obj2_tran, object_2, C2A_FIRST_CONTACT );
					if (!collide_result.Colliding()) {
						found = true;
						break;
					}

					output_space.Set_Translation(p2 + (dir * (-step * i)));
					Transform2PQP(&output_space, obj1_rot, obj1_tran);
					C2A_Collide(&collide_result, obj1_rot, obj1_tran, object_1,
								obj2_rot, obj2_tran, object_2, C2A_FIRST_CONTACT );
					if (!collide_result.Colliding()) {
						found = true;
						break;
					}
				}

				if (found) break;
			}

		}
	}

	if (!found) 
		printf("could not select the clear points; use directional method------------------------------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

	Coord3D init_config, direction;

	init_config = object_1_pose->Translation() - object_2_pose->Translation();
	init_config.Normalize();
	if (init_config.Length_Sq()==0)
	{
		init_config.X()=1.0;
	}

	direction = init_config;


	init_config *= (max_bbox_size + max_bbox_size1 );
	init_config += object_2_pose->Translation();

	bool directional_method_found = true;
	Coord3D found_pose;

	if (found || use_max_clear_conf) {
		Transform space;
		space.Set_Rotation(object_1_pose->Rotation());

		double step = (max_bbox_size)/10.0;

		for (int i = 1 ; i < 10; i++) {

			Coord3D position = ((i * step) * direction) + object_1_pose->Translation();
			space.Set_Translation(position);
			Transform2PQP(&space, obj1_rot, obj1_tran);
			C2A_Collide(&collide_result, obj1_rot, obj1_tran, object_1,
						obj2_rot, obj2_tran, object_2, C2A_FIRST_CONTACT );
			if (!collide_result.Colliding()) {
				// compare
				directional_method_found = true;
				found_pose = position;
				break;
			}

			position = ((-i * step) * direction) + object_1_pose->Translation();
			space.Set_Translation(position);
			Transform2PQP(&space, obj1_rot, obj1_tran);
			C2A_Collide(&collide_result, obj1_rot, obj1_tran, object_1,
						obj2_rot, obj2_tran, object_2, C2A_FIRST_CONTACT );
			if (!collide_result.Colliding()) {
				// compare
				directional_method_found = true;
				found_pose = position;
				break;
			}
		}
	}

	double directional_dist = 1.0e+15;
	double motion_dist = 1.0e+16;

	if (directional_method_found && found) 
	{
		
		Coord3D first_meth = found_pose - object_1_pose->Translation();
		Coord3D second_meth = output_space.Translation() - object_1_pose->Translation();
		
		directional_dist = first_meth.Length();
		motion_dist = second_meth.Length();

		if (first_meth.Length() < second_meth.Length()) 
		{
			output_space.Set_Translation(found_pose);
			output_space.Set_Rotation(object_1_pose->Rotation());

			if (!use_max_clear_conf) return;
		}
		if (!use_max_clear_conf) return;
	}

	bool found_max_clear_conf = false;

	if (use_max_clear_conf) {
		Transform space;
		space.Set_Rotation(object_1_pose->Rotation());

		double min_dist = 1.0e+17;
		int save_i = -1;

		for (int i = 0 ; i < object_2->max_clear_conf_.size() ; i++) {
			Coord3D pose_diff = object_2->max_clear_conf_[i] - object_1_pose->Translation();
			space.Set_Translation(object_2->max_clear_conf_[i]);
			Transform2PQP(&space, obj1_rot, obj1_tran);
			C2A_Collide(&collide_result, obj1_rot, obj1_tran, object_1,
						obj2_rot, obj2_tran, object_2, C2A_FIRST_CONTACT );
			if (!collide_result.Colliding()) {
				found_max_clear_conf = true;
				if ( pose_diff.Length() < min_dist) {
					min_dist =  pose_diff.Length();
					save_i = i;
				}
			}
		}

		if (min_dist < directional_dist && min_dist < motion_dist && found_max_clear_conf) {
			output_space.Set_Translation(object_2->max_clear_conf_[save_i]);
			output_space.Set_Rotation(object_1_pose->Rotation());
			return;
		}
	}

	// source
	output_space.Set_Translation(init_config); 
	output_space.Set_Rotation(object_1_pose->Rotation());
}


					  
PolyDepthReturnValue
PolyDepth(Transform*		object_1_pose,
		  C2A_Model*		object_1,
		  Transform*		object_2_pose,
		  C2A_Model*		object_2,
		  Tri*              last_triangle1,
		  Tri*              last_triangle2,
		  const std::vector<Coord3D>& object_1_vertices,
		  const std::vector<Coord3D>& object_2_vertices,
		  Transform&		output_object_1_pose, 
		  Transform&		output_object_2_pose, 
		  vector<Coord3D>&	local_penetration_depth,
		  std::vector<Coord3D>&		local_penetration_features1,
		  std::vector<Coord3D>&		local_penetration_features2,
		  Coord3D&			global_penetration_depth,
		  int&              number_of_iteration,
		  const				ClearConfigurations& clear_conf,
		  bool				use_motion_coherance,
		  bool				use_max_clear_conf,
		  PQP_REAL			absolute_contact_configuration,
		  PQP_REAL			feature_clustering_resolution)
{
#ifdef _DEBUG
	printf("\n PolyDepth \n");
#endif
	PQP_REAL toc;

	StopwatchWin32 watch;

	watch.Start();
	global_penetration_depth.Set_Value(0.0, 0.0, 0.0);

	int nItrs, NTr;
	Transform save_tr(*object_1_pose);
	Transform* moving_transform = &save_tr;

	C2A_TimeOfContactResult time_of_contact_result;

	time_of_contact_result.last_triA = last_triangle1;
	time_of_contact_result.last_triB = last_triangle2;

	Transform begin_transform_object_1;
	Transform begin_transform_object_2;
	Transform contact_configuration;


	C2A_DistanceResult distance_test_result;

	bool indep;

	int kk;

	

	int number_of_contact;

	//cje
	ContactFListIterator ContactFit;


	// for partial (local) PDs
	Coord3D cpat, cpbt;

	float am[20][20], bv[20];

	float lamv[20];

	double rel_err = 0.0, abs_err = 0.0;

	double pd_length;

	Real save_tran_vector[3];
	Real save_rot_matrix[3][3];

	Transform save_moving_transform;


	number_of_iteration=0;

	PQP_REAL temp_rot0[3][3], temp_rot1[3][3];
	PQP_REAL temp_tran0[3], temp_tran1[3];

	std::vector<Coord3D> mnkp, mnkn;

	moving_transform->Rotation().Get_Value(temp_rot0);
	object_2_pose->Rotation().Get_Value(temp_rot1);
	moving_transform->Translation().Get_Value(temp_tran0);
	object_2_pose->Translation().Get_Value(temp_tran1);


	bool new_c2a_solve = false;

	PQP_CollideResult collide_result;

	//detect penetration exist or not

	C2A_Collide(&collide_result, temp_rot0, temp_tran0, object_1,
		temp_rot1, temp_tran1, object_2, C2A_FIRST_CONTACT);

	//C2A_Distance(&distance_test_result, temp_rot0, temp_tran0, object_1,
	//	temp_rot1, temp_tran1, object_2, rel_err, abs_err);

	watch.Stop();
	printf("time to compute colliding status %f millisec\n", watch.GetTime()*1000.0);


	if (!collide_result.Colliding()) {
	//if (distance_test_result.Distance() > 0.0) {
		time_of_contact_result.collisionfree = true;

		PolyDepthReturnValue ret;

		ret.result_ = kNoPenetration;
		//ret.distance_ = distance_test_result.distance;
		ret.distance_ = 0.01;

		return ret;
	}
    //guess the free space 
	watch.Start();
	GuessContactFreeSpace(object_1_pose,
					  object_1,
					  object_2_pose,
					  object_2,
					  clear_conf,
					  use_motion_coherance,
					  use_max_clear_conf,
					  begin_transform_object_1);
	watch.Stop();
	printf("time to compute contact free space %f\n", watch.GetTime()*1000.0);

	PQP_REAL max_bbox_size = sqrt(object_2->b->d[0] * object_2->b->d[0] + 
								  object_2->b->d[1] * object_2->b->d[1] + 
								  object_2->b->d[2] * object_2->b->d[2]);

	PQP_REAL max_bbox_size1 = sqrt(object_1->b->d[0] * object_1->b->d[0] + 
								  object_1->b->d[1] * object_1->b->d[1] + 
								  object_1->b->d[2] * object_1->b->d[2]);

	if (absolute_contact_configuration < 0.0) {
		absolute_contact_configuration = min(max_bbox_size, max_bbox_size1)/ 150.0;
	}

	begin_transform_object_2.Rotation() = object_2_pose->Rotation();
	begin_transform_object_2.Translation() = object_2_pose->Translation();

	bool exit_stat = false;

	//
	// main iterative steps
	//
	while(true) {
	
	//watch.Start();
	
   number_of_iteration++;
	// Out projection steps!
	C2A_Solve( 
		&begin_transform_object_1,		// begin transformation of obj1
		moving_transform,				// end transformation of obj1
		object_1,
		&begin_transform_object_2,		// begin transformation of obj2
		object_2_pose,					// end transformation of obj2
		object_2,
		output_object_1_pose,			// contact motion of obj1
		output_object_2_pose,			// contact motion of obj2
		toc, nItrs ,NTr, 0.0, 
		time_of_contact_result);

	last_triangle1 = time_of_contact_result.last_triA;
	last_triangle2 = time_of_contact_result.last_triB;

	
	//if(time_of_contact_result.toc == 0.0)
	//{
	//	time_of_contact_result.collisionfree=1;
	//	PolyDepthReturnValue ret;

	//	ret.result_ = kNoPenetration;
	//	ret.distance_ = time_of_contact_result.distance;

	//	return ret;
	//}
	
	if (time_of_contact_result.collisionfree || !time_of_contact_result.cont_l.size()) {
		PolyDepthReturnValue ret;

		ret.result_ = kFailed;
		ret.distance_ = 0.0;

		printf("failed\n");

		return ret;
	}

	//green- toc A

	contact_configuration.Rotation() = output_object_1_pose.Rotation();
	contact_configuration.Translation() = output_object_1_pose.Translation();


	save_moving_transform.Set_Value(moving_transform->Rotation(), moving_transform->Translation());

	Coord3D temp_vec = contact_configuration.Translation() - save_moving_transform.Translation();
	pd_length = temp_vec.Length();

	kk=0;

	Transform save_contact_configuration = contact_configuration; 

	number_of_contact = time_of_contact_result.num_contact;

	if (number_of_contact <= 0 || !time_of_contact_result.cont_l.size()) {

		PolyDepthReturnValue ret;

		ret.result_ = kFailed;
		ret.distance_ = 0.0;

		printf("failed\n");

		return ret;
	}


	printf("%dth iteration, num_contacts: %d \n", number_of_iteration, number_of_contact );


	if(number_of_contact>20) {
		number_of_contact=20;
	}

	Coord3D pa1, pa2, pa3, pb1, pb2, pb3;
	Coord3D pm0, pm1, pm2, pm3, pmn, pmt;
	Coord3D prj;

	mnkp.resize(number_of_contact);
	mnkn.resize(number_of_contact);

	// feature construction
	ContactFit = time_of_contact_result.cont_l.begin();

	for (int jj = 0 ; jj < number_of_contact ; jj++) 
	{

		int cl= 3* (*ContactFit).FeatureType_A + (*ContactFit).FeatureType_B;


		if( (*ContactFit).FeatureType_A==1 ) {
			pb1=object_1_vertices[(*ContactFit).FeatureID_A[0]];
			pb1^=save_moving_transform;
		}
		else if( (*ContactFit).FeatureType_A==2 ) {
			pb1=object_1_vertices[(*ContactFit).FeatureID_A[0]];
			pb1^=save_moving_transform;

			pb2=object_1_vertices[(*ContactFit).FeatureID_A[1]];
			pb2^=save_moving_transform;
		}
		else if( (*ContactFit).FeatureType_A==3) {
			pb1=object_1_vertices[(*ContactFit).FeatureID_A[0]];
			pb1^=save_moving_transform;

			pb2=object_1_vertices[(*ContactFit).FeatureID_A[1]];
			pb2^=save_moving_transform;

			pb3=object_1_vertices[(*ContactFit).FeatureID_A[2]];
			pb3^=save_moving_transform;
		}

		if( (*ContactFit).FeatureType_B==1 ) {
			pa1=object_2_vertices[(*ContactFit).FeatureID_B[0]];
			pa1^=(*object_2_pose);
		}
		else if( (*ContactFit).FeatureType_B==2 ) {
			pa1=object_2_vertices[(*ContactFit).FeatureID_B[0]];
			pa1^=(*object_2_pose);

			pa2=object_2_vertices[(*ContactFit).FeatureID_B[1]];
			pa2^=(*object_2_pose);
		}
		else if( (*ContactFit).FeatureType_B==3 ) {
			pa1=object_2_vertices[(*ContactFit).FeatureID_B[0]];
			pa1^=(*object_2_pose);

			pa2=object_2_vertices[(*ContactFit).FeatureID_B[1]];
			pa2^=(*object_2_pose);

			pa3=object_2_vertices[(*ContactFit).FeatureID_B[2]];
			pa3^=(*object_2_pose);
		}
//edge-edge
		if ( cl == 8 ) {

			pm0=pa1-pb1;
			pm1=pa1-pb2; 
			pm2=pa2-pb1;
			pm3=pa2-pb2;

			mnkp[jj]=pm0;

			mnkn[jj]= (pm1-pm0) % (pm2-pm0);

			if(mnkn[jj]*mnkp[jj]<0) 
				mnkn[jj] *= -1.0;
			
			mnkn[jj].Normalize();
		}

//V-F
		else if( cl==6 ) {

			pm0=pa1-pb1;
			pm1=pa2-pb1;
			pm2=pa3-pb1;
 
			mnkp[jj]=pm0;

			mnkn[jj]= (pm1-pm0) % (pm2-pm0);
			if(mnkn[jj]*mnkp[jj]<0) mnkn[jj] *= -1.0;

			mnkn[jj].Normalize();
		}
//F-V
		else if( cl==10 ) {

			pm0=pa1-pb1;
			pm1=pa1-pb2;
			pm2=pa1-pb3;
			mnkp[jj]=pm0;
			mnkn[jj]= (pm1-pm0) % (pm2-pm0);

			if(mnkn[jj]*mnkp[jj]<0) mnkn[jj] *= -1.0;

			mnkn[jj].Normalize();

		}

		ContactFit++;
	}

	kk=0;
	//delete similar and degenerated contact normal
	for(int jj=0; jj<number_of_contact; jj++) {

		indep = true;
		for(int ii =0; ii<kk; ii++) {

			// inner product of normal vectors
			if(mnkn[ii]*mnkn[jj]>0.99 || mnkn[jj].Length() <= 1.0e-15) {
				indep = false;
				break;
			}
		}

		if(indep) {
			mnkn[kk]=mnkn[jj];
			mnkp[kk]=mnkp[jj];
			kk++;
		}
	}

	//
	// Projected Gauss-Seidel (In-projection)
	//
	number_of_contact = kk;
	int degen_cnt = 0;

	for (int jj = 0; jj < number_of_contact; jj++) {
		if (mnkn[jj].Length() <= 1.0e-15) {
			degen_cnt++;
			for (int kk = jj ; kk < number_of_contact-1; kk++) {
				mnkn[kk] = mnkn[kk+1];
				mnkp[kk] = mnkp[kk+1];
			}
		}
	}
	number_of_contact -= degen_cnt;
	number_of_contact -= degen_cnt;
	if (number_of_contact==1)
	{
		int a =0;
	}

	for(int i=0; i < number_of_contact; i++) {
		for(int j=0; j<number_of_contact; j++) {

			am[i][j]= mnkn[i]*mnkn[j];

			bv[i]= mnkn[i]*mnkp[i];
			lamv[i]=0;
		}
	}


	for(int i = 0; i < number_of_contact; i++){
		Real vars = 0.0;
		for(int j = 0; j < number_of_contact; j++){
			if(j != i) {
				if(lamv[j]<0) lamv[j] = 0.0;


				vars = vars + (am[i][j]*lamv[j]);

			}
		}
		lamv[i]=(bv[i] - vars)/am[i][i];
	}

	
	float tmfl=0.0, tmfl1, tmfl2, tmfl3;

	tmfl1=tmfl2=tmfl3=0;

	for(int i=0; i<number_of_contact; i++) {
		tmfl1 += mnkn[i].X() * lamv[i];
		tmfl2 += mnkn[i].Y() * lamv[i];
		tmfl3 += mnkn[i].Z() * lamv[i];
	}

	/////////////////////////////////////////// In-projection end

	//printf("number of contact %d\n", number_of_contact);
	Coord3D tmp_tran;

	tmp_tran.Set_Value( tmfl1, tmfl2, tmfl3 );
	//
	Coord3D tmp_tran2 = tmp_tran;
	tmp_tran2.Normalize();

	tmp_tran += tmp_tran2 * (absolute_contact_configuration/2.0);

	prj= save_moving_transform.Translation() + tmp_tran;

	
	temp_vec = prj - save_moving_transform.Translation();

	if (pd_length < temp_vec.Length()) {

		save_moving_transform.Rotation().Get_Value(save_rot_matrix);
		save_contact_configuration.Translation().Get_Value(save_tran_vector);

		break;
	}

	save_moving_transform.Rotation().Get_Value(save_rot_matrix);
	prj.Get_Value(save_tran_vector);


	watch.Start();
	C2A_Distance(&distance_test_result, save_rot_matrix, save_tran_vector, object_1,
				  temp_rot1,temp_tran1, object_2,rel_err, abs_err);
	watch.Stop();
	printf("time to compute distance %f\n", watch.GetTime() * 1000.0);

	if (distance_test_result.Distance() < absolute_contact_configuration) {
		if(distance_test_result.Distance() > 0.0) {
#ifdef _DEBUG
			printf("\n Contact \n");
#endif
		}
		// penetration
		else {
			number_of_iteration++;
	
			Coord3D tmtrp3= begin_transform_object_1.Translation() - moving_transform->Translation(); // s-t
			tmtrp3.Normalize();
			tmtrp3 *= absolute_contact_configuration * 1.2;
			tmtrp3 += save_contact_configuration.Translation();

			begin_transform_object_1.Translation() = tmtrp3;
			moving_transform->Translation().Set_Value(prj.X(), prj.Y(), prj.Z());
			


			C2A_Solve( 
				&begin_transform_object_1,		// begin transformation of obj1
				moving_transform,				// end transformation of obj1
				object_1,
				&begin_transform_object_2,		// begin transformation of obj2
				object_2_pose,					// end transformation of obj2
				object_2,
				output_object_1_pose,			// contact motion of obj1
				output_object_2_pose,			// contact motion of obj2
				toc, 
				nItrs ,NTr, 0.0, 
				time_of_contact_result);
			//watch.Stop();
			//printf("time to solve %f\n", watch.GetTime());

			new_c2a_solve = true;
			
			contact_configuration.Rotation() = output_object_1_pose.Rotation();
			contact_configuration.Translation() = output_object_1_pose.Translation();

			Coord3D tran_minus = begin_transform_object_1.Translation() - moving_transform->Translation(); // s-t
			tran_minus.Normalize();
			tran_minus *= 0.1;

			contact_configuration.Translation() += tran_minus;


			temp_vec = contact_configuration.Translation() - save_moving_transform.Translation();;

			if(pd_length < temp_vec.Length()) {

				save_moving_transform.Rotation().Get_Value(save_rot_matrix);
				save_contact_configuration.Translation().Get_Value(save_tran_vector);

				break;
			}

			save_moving_transform.Rotation().Get_Value(save_rot_matrix);
			begin_transform_object_1.Translation().Get_Value(save_tran_vector);
		}

	}

	// separation
	else if(number_of_iteration==10) 
		break;
	else {
		

		begin_transform_object_1.Translation().Set_Value(prj.X(), prj.Y(), prj.Z());
		continue;
	}

	break;

	} // iteration loop ends here



	// global PD

	global_penetration_depth = contact_configuration.Translation() - save_moving_transform.Translation();//save_moving_transform.Translation();

	/////////////////////////
#ifdef LOCAL_PENETRATIION


	//source: q(PD)
	begin_transform_object_1.Set_Translation(save_tran_vector);

	//target: o
	moving_transform->Translation().Set_Value(temp_tran0); 

	PQP_REAL R1[3][3];
	PQP_REAL T1[3];
	PQP_REAL R2[3][3];
	PQP_REAL T2[3];

	Transform2PQP(&begin_transform_object_1, R1, T1);
	Transform2PQP(&begin_transform_object_2, R2, T2);
	C2A_QueryContactOnly(&time_of_contact_result,  
						R1, 
						T1, 
						object_1,
						R2,
						T2, 
						object_2,
						absolute_contact_configuration);
	output_object_1_pose.Set_Translation(begin_transform_object_1.Translation());

		

	global_penetration_depth = output_object_1_pose.Translation() - save_moving_transform.Translation();
	printf("number of contact %d\n", time_of_contact_result.num_contact);

	//watch.Stop();
	//printf("time to solve %f\n", watch.GetTime());


	kk=0;

	number_of_contact = time_of_contact_result.num_contact;


	Coord3D pa1, pa2, pa3, pb1, pb2, pb3;
	Coord3D pm0, pm1, pm2, pm3, pmn, pmt;

	mnkp.resize(number_of_contact);
	mnkn.resize(number_of_contact);

	std::vector<Coord3D> cpa(number_of_contact), cpb(number_of_contact);

	ContactFit = time_of_contact_result.cont_l.begin();
	
	double subdivision_criteria = min(max_bbox_size, max_bbox_size1)/(double)feature_clustering_resolution;
	
	//subdivision_criteria *= subdivision_criteria; // square

	struct save_data {
		Coord3D feature_normal;
		Coord3D feature1;
		Coord3D feature2;

		save_data(Coord3D& n, Coord3D& f1, Coord3D& f2): feature_normal(n), feature1(f1), feature2(f2) {}
		save_data(): feature_normal(0,0,0), feature1(0,0,0), feature2(0,0,0) {}
	};

	std::map<unsigned int, save_data> feature_map;
	Coord3D box_min(1.0e+20,1.0e+20,1.0e+20);


	for (int i = 0 ; i < number_of_contact; i++) {
		cpat.Set_Value( (*ContactFit).P_A[0], (*ContactFit).P_A[1], (*ContactFit).P_A[2] );
		box_min[0] = min(cpat[0],box_min[0]);
		box_min[1] = min(cpat[1],box_min[1]);
		box_min[2] = min(cpat[2],box_min[2]);
		ContactFit++;
	}

	Coord3D normal, position;

	ContactFit = time_of_contact_result.cont_l.begin();

	for(int jj=0; jj<number_of_contact; jj++) {

		cpat.Set_Value( (*ContactFit).P_A[0], (*ContactFit).P_A[1], (*ContactFit).P_A[2] );

		Coord3D coord = cpat - box_min;
		coord = coord/subdivision_criteria;

		std::map<unsigned int, save_data>::iterator iter = feature_map.find((int)coord[2] * feature_clustering_resolution * feature_clustering_resolution + (int)coord[1] * feature_clustering_resolution + (int)coord[0]);

		if (iter != feature_map.end()) {
			ContactFit++;
			continue;
		}

		cpbt.Set_Value( (*ContactFit).P_B[0], (*ContactFit).P_B[1], (*ContactFit).P_B[2] );

		int cl= 3* (*ContactFit).FeatureType_A + (*ContactFit).FeatureType_B;

		if( (*ContactFit).FeatureType_A==1 ) {

			pb1=object_1_vertices[(*ContactFit).FeatureID_A[0]];
			pb1^=save_moving_transform;

		}
		else if( (*ContactFit).FeatureType_A==2 ) {

			pb1=object_1_vertices[(*ContactFit).FeatureID_A[0]];
			pb1^=save_moving_transform;

			pb2=object_1_vertices[(*ContactFit).FeatureID_A[1]];
			pb2^=save_moving_transform;
		}
		else if ( (*ContactFit).FeatureType_A == 3) {
			pb1=object_1_vertices[(*ContactFit).FeatureID_A[0]];
			pb1^=save_moving_transform;

			pb2=object_1_vertices[(*ContactFit).FeatureID_A[1]];
			pb2^=save_moving_transform;

			pb3=object_1_vertices[(*ContactFit).FeatureID_A[2]];
			pb3^=save_moving_transform;
		}

		if( (*ContactFit).FeatureType_B==1 ) {

			pa1=object_2_vertices[(*ContactFit).FeatureID_B[0]];
			pa1^=(*object_2_pose);

		}
		else if( (*ContactFit).FeatureType_B==2 ) {

			pa1=object_2_vertices[(*ContactFit).FeatureID_B[0]];
			pa1^=(*object_2_pose);

			pa2=object_2_vertices[(*ContactFit).FeatureID_B[1]];
			pa2^=(*object_2_pose);

		} 
		else if ( (*ContactFit).FeatureType_B == 3) {

			pa1=object_2_vertices[(*ContactFit).FeatureID_B[0]];
			pa1^=(*object_2_pose);

			pa2=object_2_vertices[(*ContactFit).FeatureID_B[1]];
			pa2^=(*object_2_pose);

			pa3=object_2_vertices[(*ContactFit).FeatureID_B[2]];
			pa3^=(*object_2_pose);
		}


		////////////////////////////////////////////////////////////////
		// EE

		if( cl==8 ) {

			pm0=pa1-pb1;
			pm1=pa1-pb2;
			pm2=pa2-pb1;
			pm3=pa2-pb2;

			position=pm0;
			normal= (pm1-pm0) % (pm2-pm0);


			if(normal*position<0) normal *= -1.0;
			normal.Normalize();

		}

		///////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////
		// VF
		else if( cl==6 ) {

			pm0=pa1-pb1;
			pm1=pa2-pb1;
			pm2=pa3-pb1;

			position=pm0;

			normal = (pm1-pm0) % (pm2-pm0);

			if(normal*position<0) normal *= -1.0;
			normal.Normalize();


		}

		////////////////////////////////////////////////////////////////
		// FV

		else if( cl==10 ) {

			pm0=pa1-pb1;
			pm1=pa1-pb2;
			pm2=pa1-pb3;

			position=pm0;

			normal = (pm1-pm0) % (pm2-pm0);


			if(normal*position<0) normal *= -1.0;

			normal.Normalize();

		}

		if (normal.Length() <= 1.0e-15) {
			ContactFit++;
			continue;
		}

		feature_map[(int)coord[2] * feature_clustering_resolution * feature_clustering_resolution + (int)coord[1] * feature_clustering_resolution + (int)coord[0]] = save_data(mnkn[jj], cpat, cpbt);
		ContactFit++;
	}

	number_of_contact = feature_map.size();

	local_penetration_depth.resize(number_of_contact);
	local_penetration_features1.resize(number_of_contact);
	local_penetration_features2.resize(number_of_contact);

	kk = 0;
	for (std::map<unsigned int, save_data>::iterator i = feature_map.begin(); i != feature_map.end() ; i++, kk++) {
		local_penetration_depth[kk] = global_penetration_depth * (i->second.feature_normal) * (i->second.feature_normal);
		local_penetration_features1[kk] = i->second.feature1;
		local_penetration_features2[kk] = i->second.feature2;
	}

#endif

	PolyDepthReturnValue ret;

	ret.result_ = kPenetration;
	ret.distance_ = 0.0;

	return ret;
}


				  
PolyDepthReturnValue
SkinDepth(Transform*		object_1_pose,
		  C2A_Model*		object_1,
		  Transform*		object_2_pose,
		  C2A_Model*		object_2,
		  const std::vector<Coord3D>& object_1_vertices,
		  const std::vector<Coord3D>& object_2_vertices,
		  Transform&		output_object_1_pose, 
		  Transform&		output_object_2_pose, 
		  vector<Coord3D>&	local_penetration_depth,
		  std::vector<Coord3D>&		local_penetration_features1,
		  std::vector<Coord3D>&		local_penetration_features2,
		  Coord3D&			global_penetration_depth,
		  PQP_REAL			absolute_contact_configuration,
		  PQP_REAL			feature_clustering_resolution)
{
#ifdef _DEBUG
	printf("\n PolyDepth \n");
#endif

	StopwatchWin32 watch;

	watch.Start();
	global_penetration_depth.Set_Value(0.0, 0.0, 0.0);


	Transform save_tr(*object_1_pose);
	Transform* moving_transform = &save_tr;

	C2A_TimeOfContactResult time_of_contact_result;

	Transform begin_transform_object_1;
	Transform begin_transform_object_2;
	Transform contact_configuration;


	C2A_DistanceResult distance_test_result;



	int kk;

	int number_of_iteration;

	int number_of_contact;

	//cje
	ContactFListIterator ContactFit;


	// for partial (local) PDs
	Coord3D cpat, cpbt;


	

	double rel_err = 0.0, abs_err = 0.0;





	Transform save_moving_transform;


	number_of_iteration=0;

	PQP_REAL temp_rot0[3][3], temp_rot1[3][3];
	PQP_REAL temp_tran0[3], temp_tran1[3];

	std::vector<Coord3D> mnkp, mnkn;

	Coord3D global_direction = moving_transform->Translation() - object_2_pose->Translation();
	global_direction.Normalize();

	moving_transform->Rotation().Get_Value(temp_rot0);
	object_2_pose->Rotation().Get_Value(temp_rot1);
	moving_transform->Translation().Get_Value(temp_tran0);
	object_2_pose->Translation().Get_Value(temp_tran1);

	PQP_REAL max_bbox_size = sqrt(object_2->b->d[0] * object_2->b->d[0] + 
								  object_2->b->d[1] * object_2->b->d[1] + 
								  object_2->b->d[2] * object_2->b->d[2]);

	PQP_REAL max_bbox_size1 = sqrt(object_1->b->d[0] * object_1->b->d[0] + 
								  object_1->b->d[1] * object_1->b->d[1] + 
								  object_1->b->d[2] * object_1->b->d[2]);

	if (absolute_contact_configuration < 0.0) {
		absolute_contact_configuration = min(max_bbox_size, max_bbox_size1)/ 150.0;
	}


	C2A_Distance(&distance_test_result, temp_rot0, temp_tran0, object_1,
				 temp_rot1, temp_tran1, object_2, rel_err, abs_err);

	if (distance_test_result.Distance() > absolute_contact_configuration) {
		PolyDepthReturnValue ret;
		ret.result_ = kNoPenetration;
		ret.distance_ = distance_test_result.Distance();
	}

	PQP_REAL global_pen_dist = absolute_contact_configuration-distance_test_result.Distance();
	global_direction *= global_pen_dist;
	global_penetration_depth = global_direction;

	PQP_REAL R1[3][3];
	PQP_REAL T1[3];
	PQP_REAL R2[3][3];
	PQP_REAL T2[3];

	Transform2PQP(&begin_transform_object_1, R1, T1);
	Transform2PQP(&begin_transform_object_2, R2, T2);
	C2A_QueryContactOnly(&time_of_contact_result,  
						R1, 
						T1, 
						object_1,
						R2,
						T2, 
						object_2,
						absolute_contact_configuration);
	output_object_1_pose.Set_Translation(begin_transform_object_1.Translation());



	kk=0;

	number_of_contact = time_of_contact_result.num_contact;


	Coord3D pa1, pa2, pa3, pb1, pb2, pb3;
	Coord3D pm0, pm1, pm2, pm3, pmn, pmt;

	mnkp.resize(number_of_contact);
	mnkn.resize(number_of_contact);

	std::vector<Coord3D> cpa(number_of_contact), cpb(number_of_contact);

	ContactFit = time_of_contact_result.cont_l.begin();
	
	double subdivision_criteria = min(max_bbox_size, max_bbox_size1)/(double)feature_clustering_resolution;
	
	//subdivision_criteria *= subdivision_criteria; // square

	struct save_data {
		Coord3D feature_normal;
		Coord3D feature1;
		Coord3D feature2;

		save_data(Coord3D& n, Coord3D& f1, Coord3D& f2): feature_normal(n), feature1(f1), feature2(f2) {}
		save_data(): feature_normal(0,0,0), feature1(0,0,0), feature2(0,0,0) {}
	};

	std::map<unsigned int, save_data> feature_map;
	Coord3D box_min(1.0e+20,1.0e+20,1.0e+20);


	for (int i = 0 ; i < number_of_contact; i++) {
		cpat.Set_Value( (*ContactFit).P_A[0], (*ContactFit).P_A[1], (*ContactFit).P_A[2] );
		box_min[0] = min(cpat[0],box_min[0]);
		box_min[1] = min(cpat[1],box_min[1]);
		box_min[2] = min(cpat[2],box_min[2]);
		ContactFit++;
	}

	Coord3D normal, position;

	ContactFit = time_of_contact_result.cont_l.begin();

	for(int jj=0; jj<number_of_contact; jj++) {

		cpat.Set_Value( (*ContactFit).P_A[0], (*ContactFit).P_A[1], (*ContactFit).P_A[2] );

		Coord3D coord = cpat - box_min;
		coord = coord/subdivision_criteria;

		std::map<unsigned int, save_data>::iterator iter = feature_map.find((int)coord[2] * feature_clustering_resolution * feature_clustering_resolution + (int)coord[1] * feature_clustering_resolution + (int)coord[0]);

		if (iter != feature_map.end()) {
			ContactFit++;
			continue;
		}

		cpbt.Set_Value( (*ContactFit).P_B[0], (*ContactFit).P_B[1], (*ContactFit).P_B[2] );

		int cl= 3* (*ContactFit).FeatureType_A + (*ContactFit).FeatureType_B;

		if( (*ContactFit).FeatureType_A==1 ) {

			pb1=object_1_vertices[(*ContactFit).FeatureID_A[0]];
			pb1^=save_moving_transform;

		}
		else if( (*ContactFit).FeatureType_A==2 ) {

			pb1=object_1_vertices[(*ContactFit).FeatureID_A[0]];
			pb1^=save_moving_transform;

			pb2=object_1_vertices[(*ContactFit).FeatureID_A[1]];
			pb2^=save_moving_transform;
		}
		else if ( (*ContactFit).FeatureType_A == 3) {
			pb1=object_1_vertices[(*ContactFit).FeatureID_A[0]];
			pb1^=save_moving_transform;

			pb2=object_1_vertices[(*ContactFit).FeatureID_A[1]];
			pb2^=save_moving_transform;

			pb3=object_1_vertices[(*ContactFit).FeatureID_A[2]];
			pb3^=save_moving_transform;
		}

		if( (*ContactFit).FeatureType_B==1 ) {

			pa1=object_2_vertices[(*ContactFit).FeatureID_B[0]];
			pa1^=(*object_2_pose);

		}
		else if( (*ContactFit).FeatureType_B==2 ) {

			pa1=object_2_vertices[(*ContactFit).FeatureID_B[0]];
			pa1^=(*object_2_pose);

			pa2=object_2_vertices[(*ContactFit).FeatureID_B[1]];
			pa2^=(*object_2_pose);

		} 
		else if ( (*ContactFit).FeatureType_B == 3) {

			pa1=object_2_vertices[(*ContactFit).FeatureID_B[0]];
			pa1^=(*object_2_pose);

			pa2=object_2_vertices[(*ContactFit).FeatureID_B[1]];
			pa2^=(*object_2_pose);

			pa3=object_2_vertices[(*ContactFit).FeatureID_B[2]];
			pa3^=(*object_2_pose);
		}


		////////////////////////////////////////////////////////////////
		// EE

		if( cl==8 ) {

			pm0=pa1-pb1;
			pm1=pa1-pb2;
			pm2=pa2-pb1;
			pm3=pa2-pb2;

			position=pm0;
			normal= (pm1-pm0) % (pm2-pm0);


			if(normal*position<0) normal *= -1.0;
			normal.Normalize();

		}

		///////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////
		// VF
		else if( cl==6 ) {

			pm0=pa1-pb1;
			pm1=pa2-pb1;
			pm2=pa3-pb1;

			position=pm0;

			normal = (pm1-pm0) % (pm2-pm0);

			if(normal*position<0) normal *= -1.0;
			normal.Normalize();


		}

		////////////////////////////////////////////////////////////////
		// FV

		else if( cl==10 ) {

			pm0=pa1-pb1;
			pm1=pa1-pb2;
			pm2=pa1-pb3;

			position=pm0;

			normal = (pm1-pm0) % (pm2-pm0);


			if(normal*position<0) normal *= -1.0;

			normal.Normalize();

		}

		if (normal.Length() <= 1.0e-15) {
			ContactFit++;
			continue;
		}

		feature_map[(int)coord[2] * feature_clustering_resolution * feature_clustering_resolution + (int)coord[1] * feature_clustering_resolution + (int)coord[0]] = save_data(mnkn[jj], cpat, cpbt);
		ContactFit++;
	}

	number_of_contact = feature_map.size();

	local_penetration_depth.resize(number_of_contact);
	local_penetration_features1.resize(number_of_contact);
	local_penetration_features2.resize(number_of_contact);


	kk = 0;
	for (std::map<unsigned int, save_data>::iterator i = feature_map.begin(); i != feature_map.end() ; i++, kk++) {
		local_penetration_depth[kk] = global_penetration_depth * (i->second.feature_normal) * (i->second.feature_normal);
		local_penetration_features1[kk] = i->second.feature1;
		local_penetration_features2[kk] = i->second.feature2;
	}

	PolyDepthReturnValue ret;

	ret.result_ = kPenetration;
	ret.distance_ = 0.0;

	return ret;
}




bool FindMaximallyClearConfigure(const std::vector<Coord3D>& object_vertices, 
								 C2A_Model* object, 
								// ClearConfigurations& maximally_clear_conf,
								 bool solid,
								 int grid_resolution) 
{

	kMaxClearConf.resize(0);

	std::vector<Coord3D> ifconf(grid_resolution);
	std::vector<PQP_REAL> ifcdis(grid_resolution);
	std::vector<bool> ifcfac(grid_resolution);

	C2A_Model* single_triangle = new C2A_Model();

	single_triangle->BeginModel();

	PQP_REAL p1[3], p2[3], p3[3];

	PQP_REAL scale = 1.0e-10;

    p1[0] = 0.1000 * scale;
	p1[1] = 0.2000 * scale;
	p1[2] = 0.1000 * scale;

	p2[0] = -0.2000 * scale;
	p2[1] = -0.1000 * scale;
	p2[2] = 0.1000 * scale;

    p3[0] = 0.1000 * scale;
	p3[1] = -0.1000 * scale;
	p3[2] = -0.2000 * scale;

	single_triangle->AddTri(p1,p2,p3,0,0,1,2);
  
	single_triangle->EndModel();  

	Coord3D bmin, bmax;

	C2A_DistanceResult dist_result;

	// bounding box for obj. 1
	bmin.Set_Value(1.0e+15, 1.0e+15, 1.0e+15);
	bmax.Set_Value(-1.0e+15, -1.0e+15, -1.0e+15);

	for (int i = 0; i < object_vertices.size(); i++)
	{
		if( bmin.X() > object_vertices[i].X() ) {
			bmin.Set_X(object_vertices[i].X()); 
		}

		if( bmin.Y() > object_vertices[i].Y() ) {
			bmin.Set_Y(object_vertices[i].Y()); 
		}

		if( bmin.Z() > object_vertices[i].Z() ) {
			bmin.Set_Z(object_vertices[i].Z()); 
		}

		if( bmax.X() < object_vertices[i].X() ) {
			bmax.Set_X(object_vertices[i].X()); 
		}

		if( bmax.Y() < object_vertices[i].Y() ) {
			bmax.Set_Y(object_vertices[i].Y()); 
		}

		if( bmax.Z() < object_vertices[i].Z() ) {
			bmax.Set_Z(object_vertices[i].Z()); 
		}
	}

	Coord3D interval = bmax - bmin; 

	PQP_REAL unitt= ( interval.X() * interval.Y() * interval.Z() ) / grid_resolution;

	unitt = pow(unitt, 0.3333333);

	PQP_REAL rot[3][3];
	PQP_REAL tran[3];

	int i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			if(i==j) rot[i][j]=1.0;
			else rot[i][j]=0.0;
		}
		tran[i]=0.0;
	}

	PQP_REAL move[3];

	i = 0;
	j = 0;
	PQP_REAL rel_err = 0.0, abs_err = 0.0;
	PQP_REAL tmdb1, tmdb2;
	Coord3D tmtrp1;

	for (move[0] = bmin.X(); move[0] < bmax.X(); move[0] = move[0]+unitt) {
		for (move[1] = bmin.Y(); move[1] < bmax.Y(); move[1] = move[1]+unitt) {
			for (move[2] = bmin.Z(); move[2] < bmax.Z(); move[2]  = move[2]+unitt) {

				C2A_Distance(&dist_result, 
					rot, 
					move, 
					single_triangle, 
					rot, 
					tran, 
					object, 
					rel_err, 
					abs_err);

				i++;


				tmdb1 = dist_result.Distance();
				//printf("closest distance triangle %d %d\n", dist_result.t1, dist_result.t2);

				PQP_REAL insideness = 1.0;

				if (solid) {

					Coord3D pp1(object->GetTriangle(dist_result.t2)->p1);
					Coord3D pp2(object->GetTriangle(dist_result.t2)->p2);
					Coord3D pp3(object->GetTriangle(dist_result.t2)->p3);
					Coord3D t1(move);

					Coord3D normal = (pp2-pp1) % (pp3-pp1);
					Coord3D pnt_direction = pp1-t1;
					insideness = normal*pnt_direction;
				}

				if (i == ifcdis.size()) {
					ifcdis.resize(i+1);
					ifconf.resize(i+1);
					ifcfac.resize(i+1);
				} 


				if (tmdb1 > unitt * 2.0 && insideness > 0.0) {

					ifcdis[j] = tmdb1;
					ifconf[j] = move;
					ifcfac[j] = true;

					for (int k = 0 ; k < j ; k++) {

						if (ifcfac[j] || ifcfac[k]) {

							tmdb2=ifconf[j].Dist(ifconf[k]);

							if(ifcdis[k]+unitt*0.9<ifcdis[j]) {

								if(tmdb2<ifcdis[j]) {

									ifcfac[k]=false;
								}
							}
	
							else if(ifcdis[j]+unitt*0.9<ifcdis[k]) {

								if(tmdb2<ifcdis[k]) {

									ifcfac[j]=false;
								}
							}

						}

					}
					j++;
				}
			}
		}
	}

	ifconf.resize(j);
	ifcdis.resize(j);
	ifcfac.resize(j);

	// 남아있는 configuration 들 사이의 비교를 통한 제거
	// 두 개의 configuration 의 거리값 중 큰 것이 두 configuration 사이의 거리값보다 크면 거리값이 작은 configuration을 제거한다

	for (j = 0; j < ifconf.size() ; j++)
	{
		if(ifcfac[j]) {


			for (int k = 0; k < j; k++) {

				/////
				if(ifcfac[j] || ifcfac[k]) {

					tmdb2=ifconf[j].Dist(ifconf[k]);


					if(ifcdis[k]<ifcdis[j]) {

						if(tmdb2<ifcdis[j]) {
							ifcfac[k]=0;
						}
					}
					else {
						if(tmdb2<ifcdis[k]) {
							ifcfac[j]=0;
						}
					}
				}
			}
		}
	}

	tmdb2 = sqrt(unitt * unitt * 2);

	// AABB의 8개 꼭지점 근처에 있는 configuration들을 제거한다
	for (j = 0; j < ifconf.size(); j++)
	{
		if(ifcfac[j]) {

			//
			tmtrp1.Set_Value( bmin.X(), bmin.Y(), bmin.Z() );
			tmdb1=ifconf[j].Dist(tmtrp1);
			if(tmdb1<tmdb2) {
				ifcfac[j]=false;
			}
			//
			tmtrp1.Set_Value( bmin.X(), bmin.Y(), bmax.Z() );
			tmdb1=ifconf[j].Dist(tmtrp1);
			if(tmdb1<tmdb2) {
				ifcfac[j]=false;
			}
			//
			//
			tmtrp1.Set_Value( bmin.X(), bmax.Y(), bmin.Z() );
			tmdb1=ifconf[j].Dist(tmtrp1);
			if(tmdb1<tmdb2) {
				ifcfac[j]=false;
			}
			//
			tmtrp1.Set_Value( bmin.X(), bmax.Y(), bmax.Z() );
			tmdb1=ifconf[j].Dist(tmtrp1);
			if(tmdb1<tmdb2) {
				ifcfac[j]=false;
			}
			//
			//
			//
			tmtrp1.Set_Value( bmax.X(), bmin.Y(), bmin.Z() );
			tmdb1=ifconf[j].Dist(tmtrp1);
			if(tmdb1<tmdb2) {
				ifcfac[j]=false;
			}
			//
			tmtrp1.Set_Value( bmax.X(), bmin.Y(), bmax.Z() );
			tmdb1=ifconf[j].Dist(tmtrp1);
			if(tmdb1<tmdb2) {
				ifcfac[j]=false;
			}
			//
			//
			tmtrp1.Set_Value( bmax.X(), bmax.Y(), bmin.Z() );
			tmdb1=ifconf[j].Dist(tmtrp1);
			if(tmdb1<tmdb2) {
				ifcfac[j]=false;
			}
			//
			tmtrp1.Set_Value( bmax.X(), bmax.Y(), bmax.Z() );
			tmdb1=ifconf[j].Dist(tmtrp1);
			if(tmdb1<tmdb2) {
				ifcfac[j]=false;
			}
		}
	}


	for (i = 0; i < ifconf.size(); i++)
	{
		if(ifcfac[i]) {
			kMaxClearConf.push_back(ifconf[i]);
			//kMaxClearConfDist.push_back(ifcdis[i]);
			//maximally_clear_conf.AddClearConfiguration(ifconf[i],ifcdis[i]);
		}
	}

	printf("number of maximally clear configuration: %d\n", kMaxClearConf.size());

	delete single_triangle;

	object->max_clear_conf_ = kMaxClearConf;

	if (kMaxClearConf.size()) return true;
	return false;
}

