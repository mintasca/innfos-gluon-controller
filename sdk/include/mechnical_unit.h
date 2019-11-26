#ifndef MECHNICAL_UNIT_H
#define MECHNICAL_UNIT_H


#include "base.h"
#include "database.h"
#include "actuatorcontroller.h"

class AccDecDate
{
public:
	double vel;
	double acc;
	double jerk;
	void SetParameter(double v,double a,double jerk);
};

enum RobotStatus
{
	ON = 0,
	OFF = 1,
	ABNORMAL = 2,
};

enum RobotType
{
	GL_4L3,
	GL_6L3,
	GL_2P8_P6_3P3,
	GL_6L6,
	GL_2L6_4L3,
	LINK_MECHANISM,
	FREE_COMBINATION,
};

enum ActuatorType
{
	QDD_LITE_PR60_36,
	QDD_LITE_NE30_36,
	QDD_LITE_EL20_36,
	QDD_PR60_36,
	QDD_NE30_36,
	QDD_EL20_36,
	QDD_PRO_NU80_100_110,
	QDD_PRO_PR60_80_90,
	QDD_PRO_NE30_50_70,
	ACTUATOR_FREE,
};

class LinkUnit
{
private:

	uint8_t id_; //sca id
	ActuatorType type_;//sca type

	//automatic allocation
	ActuatorController *pointer_;

	//get from sca type
	double reduction_rate_;
	double kt_;//torque constant

	int direct_; //move direct 1:-1 default:1 

	//sca parameters,get from sca
	double max_motor_speed_;//transfer to axis
	double min_soft_limit_;
	double max_soft_limit_;

	//int joint_type_;//0:revolute, 1:prismatic defult:0
	//double home_position_;
	//dynamics 
	//double m_; //mass
	//double p_[3];// position of center-of-mass(in the link coordinantes)	
	//double Ixx_;//inertia Ixx
	//double Ixy_;//inertia Ixy
	//double Ixz_;//inertia Ixz
	//double Iyy_;//inertia Iyy
	//double Iyz_;//inertia Iyz
	//double Izz_;//inertia Izz
	//double fv_;// viscous friction
	//double fc_; //coulomb friction
	//double Jm_;//motor inertia 

public:
	AccDecDate joint_speed_;

	//kinematics MDH parameters
	double theta_;
	double alpha_;
	double a_;
	double d_;

	LinkUnit();
	int InitActuator(uint8_t id,ActuatorType type_);
	uint8_t GetActuatorId();

	void SetActuatorPointer(ActuatorController *pointer);
	void SetMdhParameters(double theta,double alpha,double a,double d);
	void SetSpeedData(double vel,double acc,double jerk);
	void SetDirect(int direct);
	int GetActuatorData();

	void RequestCVPValue();
	void GetActuatorCVPFast(double *current,double *vel,double *pos);
	void GetActuatorTVPFast(double *tau,double *vel,double *pos);
	double GetActuatorPositionFast();
	double GetActuatorPositionSlow();

	void SetActuatorTorque(double tau);
	void SetActuatorPosition(double pos);
	void SetActuatorHomingPosition();
	void SetActuatorLimitData(double min_pos, double max_pos);
	void SetLockEnergy(double energy);
	void SetActuatorPositionPidData(double kp,double ki,double kd);	
	void SetActuatorVelocityPiData(double kp,double ki);

	uint32_t GetErrorCode();
	void ClearActuatorError();
	int CheckPosition(double joint,double last_joint,double sample);
	int CheckTorque(double tau);
	void DisableActuator();
	bool SaveActuatorParameters();
	~LinkUnit();
};

class Robot
{
private:
	int axis_num_;
	RobotType type_;
	AccDecDate rot_speed_;
	AccDecDate line_speed_;
	vector <uint8_t> idArray;
	ActuatorController *pController;
	LinkUnit link_data_[MAX_AXIS_NUM];
	vector< ActuatorController::UnifiedID > unified_id_array_;

	int GetParametersFromActuator();
	int GetParametersFromDatabase();	
	int AddLinkUnit(LinkUnit * link_unit,int axis_num);
public:

	Robot();
	int InitRobot(LinkUnit *link_unit,int axis_num);
	int ServerInit(ActuatorController * pointer_controller,
	vector <uint8_t>	 id_vec,vector< ActuatorController::UnifiedID > uid_vec);

	int GetRobotJointNum();
	int GetRobotType();
	uint32_t GetErrorCode();
	void GetCurrentCVPFast(double current[],double vel[],double pos[]);
	void GetCurrentTVPFast(double tau[],double vel[],double pos[]);
	void GetCurrentMachineJoint(double angle[]);
	void GetCurrentMachineJointFast(double angle[]);
	void GetJointSpeedAccDecParameters(AccDecDate *joint_speed);
	void GetLineSpeedAccDecParameters(AccDecDate *line_speed);
	void GetRotateSpeedAccDecParameters(AccDecDate *rot_speed);

	void DisableRobot();
	int SaveParameters();
	void ActivateHomingMode();
	void ActivateCurrentMode();
	void ActivateVelocityMode();
	void ActivatePositionMode();
	void RequestCVPValue();
	void AddParaRequestCallback(ActuatorController::doubleFuncPointer callback);
	void ClearAllActuatorCallbacks();

	int SendMachinePosition(double joint[],double last_joint[]);
	int SetMachineTorque(double tau[]);
	int SendPosition(double joint[],double last_joint[]);
	int SetPosition(double joint[]);	
	int SetHomingPosition();
	int SetLimitData(double min_pos[],double max_pos[]);	
	int SetLockEnergy(double energy[]);
	int SetPositionPidData(double kp[],double ki[],double kd[]);
	int SetVelocityPiData(double kp[],double ki[]);

	int ForwardKinematics(double q[],double pose[]);
	int InverseKinematics(double q[],double pose[]);
	int ForwardDynamics();
	int InverseDynamics(double q[],double qd[],double qdd [],double tau[]);
	int GravityCompensation(double q[],double qd[]);
	int CollisionDetection(double last_q[],double q[],double qd[],double tau[],double w_direct[3]);

	~Robot();

};

//class Mechniancl;
//Group

void DisableAllActuator();
void InitC();
Robot* GetCurrentRobot();
void SetRealTimePos(double value[]);
void GetRealTimePos(double value[]);

ResourceData * GetResourceDate();

#endif
