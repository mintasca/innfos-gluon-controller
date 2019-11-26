#ifndef EXECUTE_H
#define EXECUTE_H

#include <string>
#include <list>

#include "base.h"
#include "interpolation.h"

using namespace std;

enum OperationCycleMode
{ 
	STEP,
	CYCLE,
	CONTINUOUS,
};

//int MOVE_instruction(char *line,int num,double pos[],path_type *type);

class TimerInstructionData
{
public: 
	int time_; // wait to add var
	void ExecuteInstruction();
	int GetTimerInstructionData(char *line);
};


class MoveInstructionData
{
public: 
	MoveType type_;
	int joint_num_; // <=2*MAX_AIXS_NUM ==real_axis_num
	double joint_[2*MAX_AXIS_NUM];
	SpeedData speed_data_;
	ZoneData zone_data_;	
	int GetMoveInstructionData(char *line);
};

class DoutInstructionData
{
public:
	int index_;//IO index
	int status_; // on/off
};

class InsturctionUnion
{
public:
	TimerInstructionData timer_;
	MoveInstructionData move_;
	DoutInstructionData dout_;
};

enum InstructionType
{ 
	MoveInstruction,
	TimerInstruction,
	DoutInstruction,

//	IOInstruction,
//	ControlInstruction,
//	ShiftInstruction,
//	OperateInstruction,
};

	
class InstructionUnit
{
public:
	int line_num;
	InstructionType type_;
	InsturctionUnion instru_;
	int DecodeInstruction(char *line);	
};

int DecodeInstructionFile(string file,     list<InstructionUnit> *buffer);

int RunningProgram(int argc, char *argv[]);
int Tutorials();
int ExecuteFileInstruction(Robot *unit,string file_name);


#endif


