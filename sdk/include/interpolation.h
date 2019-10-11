#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <../include/base.h>
#include <../include/speed_profile.h>
#include <../include/mechnical_unit.h>

#include "actuatorcontroller.h"


class JointPath
{
private:
	double start_joint_;
	double target_joint_;
	double vel_;
	double acc_;
	double dec_;
	double jerk_;
	double direct_;
	double displacement_;
public:
	JointPath();
	SpeedProfile profile_;
	void SetParameters(double start_joint,double target_joint,double V,double A,double D,double J);
	void InterpolationPreprocess();
	double Interpolation(double t);
	~JointPath();
};

class LinearPath
{
private:
	double start_pose_[7];
	double target_pose_[7];

	double vel_line_;
	double acc_line_;
	double dec_line_;
	double jerk_line_;

	double vel_rot_;
	double acc_rot_;
	double dec_rot_;
	double jerk_rot_;

	double angle_;
	double displacement_;
	double delta_pos_[3];

public:
	LinearPath();
	int type_;//0 trans 1 rot
	SpeedProfile profile_trans_;
	SpeedProfile profile_rot_;
	double tall_;
	double tacc_;
	double tdec_;

	void SetParameters(double start_pose [], double target_pose[],double vl,double al,double dl,double jl,double vr,double ar,double dr,double jr);
	void InterpolationPreprocess();
	int Interpolation(double t, double pose[]);
	~LinearPath();
	
};

class CircularPath
{
private:
	double start_pose_[7];
	double aux_pose_[7];
	double target_pose_[7];

	double vel_line_;
	double acc_line_;
	double dec_line_;
	double jerk_line_;

	double vel_rot_;
	double acc_rot_;
	double dec_rot_;
	double jerk_rot_;

	double angle_;//ori
	double displacement_;
	double delta_pos_[3];
	double frame_[3][3];
	double circle_center_[3];
	double radius_;
	double central_angle_[2];
	int direct_;
	double theta_;
public:
	CircularPath();
	double tall_;
	double tacc_;
	double tdec_;
	SpeedProfile profile_trans_;
	SpeedProfile profile_rot_;
	int type_;//0 trans 1 rot

	void SetParameters(double start_pose[],double aux_pose[],double target_pose[],
		double vl,double al,double dl,double jl,double vr,double ar,double dr,double jr);
	void InterpolationPreprocess();
	int Interpolation(double t, double pose[]);
	~CircularPath();
};

enum MoveType
{ 
	MoveJoint,
	MoveLinear,
	MoveCircular,
};

class SpeedData
{
	int level_;
public:
	SpeedData(); 
	~SpeedData(); 
	double v_ori_;
	double v_tcp_;
	double vj_;	
	int GetInstructionSpeedData(char *speed_level);
};
class ZoneData
{
public:
	int level_;
	double percent_;
	double distance_;	
	int GetInstructionZoneData(char * zone_char);
};

class SpeedOverride
{
public:
	double vel_;
	double acc_;
	double dec_;
	double jerk_;
};

class InterpolationData
{
private:

public:
	int line_num_;
	MoveType type_;
	JointPath jpath_[MAX_AXIS_NUM];
	LinearPath lpath_;
	CircularPath cpath_;

	AccDecDate joint_speed_[MAX_AXIS_NUM];
	AccDecDate line_speed_;
	AccDecDate rot_speed_;

	SpeedData speed_data_;

	ZoneData zone_data_;

	double tleft_;
	double tright_;
	double tall_;

	double start_joint_[MAX_AXIS_NUM];
	double aux_joint_[MAX_AXIS_NUM];
	double target_joint_[MAX_AXIS_NUM];

	int MovePreProcess(Robot *unit);	
	int MovePathFlow(Robot *unit,double joint [ ],double t);
};


void InitSpeedOverride();
void SetSpeedOverride(double v);

int RecordSinglePointInDragTeachMode(Robot *unit);
int ReplayPathStep(Robot *unit,InterpolationData *path_list,int count);
int ReplayPathList(Robot *unit,InterpolationData *path_list,int count);
int ReplayPathListSmooth(Robot *unit,InterpolationData *path_list,int count);

int RecordContinuousPointInDragTeachMode(Robot *unit);
int ReplayTrajectory(Robot *unit);

int MoveExcitationTrajectories(Robot *unit);

int MoveJointPath(Robot *unit,double start_joint[],double target_joint[],AccDecDate *speed);

#endif
