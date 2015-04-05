#pragma pack(push,1)

typedef struct{
	float	x;
	float	y;
	float	theta;
}arobotpose;

typedef struct{
	float theta;
	float dist;
	int exitRobot;
}roboControl;

typedef struct{
	float theta;
	float x;
	float z;
	int move;
}lenPoseWorld;

typedef struct{
	float theta;
	float x;
	float z;
	int move;
}kaitoPoseWorld;

#pragma pack(pop)