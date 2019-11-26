#ifndef DATABASE_H
#define DATABASE_H

#include "base.h"

class ResourceData
{
public:
	bool free_force_switch_;//0:off 1:on
	double free_force_factor_;//0.5 - 1.5
	int type_;
	double joint_max_speed_[MAX_AXIS_NUM];
	double joint_max_acc_[MAX_AXIS_NUM];
	double joint_max_jerk_[MAX_AXIS_NUM];

//	double motor_max_speed_[MAX_AIXS_NUM];

//	double joint_min_soft_limit_[MAX_AIXS_NUM];
//	double joint_max_soft_limit_[MAX_AIXS_NUM];

	double line_max_speed_;
	double line_max_acc_;
	double line_max_jerk_;
	double rotate_max_speed_;
	double rotate_max_acc_;
	double rotate_max_jerk_;
	double reduction_gear_ratio_[MAX_AXIS_NUM];
	double rotation_direction_[MAX_AXIS_NUM];
	double dh_[4*MAX_AXIS_NUM];
	double dynamics_ident_para_[13*MAX_AXIS_NUM];
	double fourier_series_[11*MAX_AXIS_NUM];

};

//#include <string>
//int UpdateRobotType(string type);
int DynamicsSwitch(char *type);
int UpdateRobotType(char *type,int id);

int SelectFromTableList(ResourceData *res_data);

#endif
