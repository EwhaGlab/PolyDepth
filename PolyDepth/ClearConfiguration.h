#ifndef __ClearConfiguration_h
#define __ClearConfiguration_h

#include <vector>

#include <C2A/LinearMath.h>



class ClearConfiguration {

public:

	ClearConfiguration(): position_(0,0,0), distance_to_object_(-1.0), used_(false) 
	{
	}

	ClearConfiguration(const Coord3D& pos, PQP_REAL dist = -1.0): position_(pos), distance_to_object_(dist), used_(true)
	{
	}


	ClearConfiguration(const ClearConfiguration& MCC): 
		position_(MCC.position_), 
		distance_to_object_(MCC.distance_to_object_),
		used_(MCC.used_)
	{
	}

	Coord3D position() const { return position_; }
	PQP_REAL distance_to_object() const { return distance_to_object_; }

	bool HasValidDistance() const { return !(distance_to_object_ < 0.0); }
	bool used() const { return used_; }

private:

	Coord3D		position_;

	PQP_REAL	distance_to_object_;

	bool		used_;
};

class ClearConfigurations {

public:

	ClearConfigurations() {}
	ClearConfigurations(const ClearConfigurations& a)
	{
		for (int i = 0 ; i < 3 ; i++) {
			configurations_[i] = a.configurations_[i];
		}
	}

	void AddClearConfiguration(const Coord3D& pos)
	{
		for (int i = 0 ; i < 3 ; i++) {

			if (configurations_[i].used()) {
				if (pos.Dist(configurations_[i].position()) < 1.0e-12) {
					return;
				}
			}
			else {
				configurations_[i] = ClearConfiguration(pos);
				return;
			}
		}

		configurations_[0] = configurations_[1];
		configurations_[1] = configurations_[2];
		configurations_[2] = ClearConfiguration(pos);
	}

	void AddClearConfiguration(const Coord3D& pos, double dist)
	{
		for (int i = 0 ; i < 3 ; i++) {

			if (configurations_[i].used()) {
				if (pos.Dist(configurations_[i].position()) < 1.0e-12) {
					return;
				}
			}
			else {
				configurations_[i] = ClearConfiguration(pos, dist);
				return;
			}
		}

		configurations_[0] = configurations_[1];
		configurations_[1] = configurations_[2];
		configurations_[2] = ClearConfiguration(pos, dist);
	}

	ClearConfiguration& operator[](int i) { return configurations_[i]; }

	const ClearConfiguration& operator[](int i) const { return configurations_[i]; }

	int size() const { return 3; }

public:
	
	ClearConfiguration configurations_[3];
};

#endif