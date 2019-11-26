#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include "base.h"
#include "speed_profile.h"
#include "mechnical_unit.h"
#define TRAGECTORY_FILE "file/trajectory.txt"

//#include "actuatorcontroller.h"

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


int decode_char_line(const char *line,double pos[],int num);

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

class SpeedOverride
{
public:
	double vel_;
	double acc_;
	double dec_;
	double jerk_;
};

void InitSpeedOverride();
void SetSpeedOverride(double v);

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


int MoveJointPath(Robot *unit,double start_joint[],double target_joint[],AccDecDate *speed);
int MoveToSpecifiedPosition(double target_joint[]);
int MoveToHomePosition();
int MoveToStablePosition();

int MoveLinearPath(Robot *unit,double start_joint[],double target_joint[],AccDecDate *line_speed,AccDecDate *rot_speed);
int MoveCircularPath(Robot *unit,double start_joint[],double aux_joint[],double target_joint[],AccDecDate *line_speed,AccDecDate *rot_speed);
int MovePathList(Robot *unit,InterpolationData *path_list,int count);
int MovePathListSmooth(Robot *unit,InterpolationData *path_list,int count);
int MoveTrajectory(Robot *unit,string file_name,bool infinite_flag);
int MoveExcitationTrajectories(Robot *unit,int iter_num);
int ReplayPathStep(Robot *unit,InterpolationData *path_list,int count);

int RecordSinglePointInDragTeachMode(Robot *unit,string file_name);
int RecordContinuousPointInDragTeachMode(Robot *unit,string file_name);

//int SaveMatrixToFile(string file_name,double matrix[],int row,int column);
bool IsTerminal();
void SetTerminalFlag();
int GetReplayOverrideFromTerminal();
int SetSpeedOverride();
void SetReplayOverride(double value);

//void InitStopFlag();
//void SetStopFlag();




#endif
