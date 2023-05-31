/*
auther: kaming
file name: RAG2_kinematic.h
project: forward and inverse kinematics for a RAG2 robotic arm
*/

#ifndef __RAG2_KINEMATIC_H
#define __RAG2_KINEMATIC_H


#include <math.h>
#include <iostream>
using namespace std;

#define PI 3.14159265


// joint state value for forward kinematic calculation (FK input) 
// it should be in radian. every thing here is radian
// the angle is limit from -PI to PI !
float joint_angle[6]={(70*PI)/180, (-70*PI)/180,(-20*PI)/180,(50*PI)/180,(-80*PI)/180,(20*PI)/180};

//testing// current angle
float cur_angle[6]={(-100*PI)/180, (-60*PI)/180,(-20*PI)/180,(50*PI)/180,(-80*PI)/180,(20*PI)/180};
//testing//


// Parameters for DH convention (i-1 th ahlpe, i-1 th a, i th d, i th theta)
// this martic give the info of robot structure and state 
// the joint 7 is just a translation from wrist coord to tools coord
float DH_value [7][4]={ {0,0,0,joint_angle[0]},
                        {-0.5*PI,0,0,joint_angle[1]},
                        {0,250,0,joint_angle[2]},
                        {-0.5*PI,0,250,joint_angle[3]},
                        {0.5*PI,0,0,joint_angle[4]},
                        {-0.5*PI,0,0,joint_angle[5]},
                        {0,0,150,0}};

// Result of forward kinematic. (FK output)
//it give a martic that give info of the tools coordinate and orientation
float fk_matric_result [4][4];

// tools coordinate martic for inverse kinematic (IK input)
float IK_matric [4][4]={{0.422163,      -0.757678,      0.497698,       557 },
                        {-0.832499,     -0.541327,      -0.117944,      0       },
                        {0.358781,      -0.364541,      -0.859294,      0        },
                        {0,       0,       0,       1}};

// translation matrix from tool coord to wrist coord 
float inv_Joint_7_matrix [4][4]={   {   1,  0,  0,  0},
                                    {   0,  1,  0,  0},
                                    {   0,  0,  1,  -150},
                                    {   0,  0,  0,  1}};




// Result of inverse kinematic. (IK output)
// it give <=8 different result of the joint value of a given state
float joint_angle_result [8][6];


// array that indicate which IK result is valid
bool valid_result[8]={true,true,true,true,true,true,true,true};

// optimal solution index
int optimal_index=-1;

// 4x4 matric multiply
void multiply(float mat1[4][4], float mat2[4][4],float res[4][4]); 

// init or reset_IK_reuslt_martic 
void reset_fk_matric ();

// init or reset_IK_reuslt_martic
void reset_ik_matric ();

// forward kinematic calculation
void FK_calculation (float joint_angle[6]);

// inverse kinematic calculation
void IK_calculation(float IK_matric [4][4]);

// construction of translation_matric for IK input. using the x-y-z fixed angle systems or z-y-x Euler angle systems
// x-y-z fixed coordinate is mean by first rotate x axis by Rx then y-axis by Ry then z-axis by Rz. IN fixed coord To perform rotation of a coordinate
// Z-Y-X Euler coordinate is mean by first rotate Z-axis by Rz then y-axis by Ry then X-axis by Rx. IN Rotating coord To perform rotation of a coordinate
// the last three parameter is is the tranlation from A to B by x-asix x mm then y asix y mm then z asix z mm
void translation_matric(float IK_matric [4][4], float Rx,float Ry,float Rz,float x,float y,float z);

// construction of translation_matric for IK input. using the Euler parameter/ Quaternion
// Please reference to the bood and https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
// i did not test the matrix currently. Need to be tested!
void translation_matric(float IK_matric [4][4], float qx,float qy,float qz,float qw,float x,float y,float z);

// For giving input coordinate, deduct that the coord is it reachable
// Dextruos Workspace r <= 350mm return 0
// Reachable workspace r <= 650mm return 1
// unreachable return 2
int reaching_space(float IK_matric [4][4]);

// As the joint state init in the calculation is differnt from the real world, a translation is required
// transform the joint state in virtual world(drive by DH value) to real world robot
void V2R_transform (float virtual_angle[6],float real_angle[6]);

// transform the joint state in real world to virtual world (drive by DH value)
void R2V_transform (float virtual_angle[6],float real_angle[6]);

// calculate the vaild result and return the best result index
// return 0 to 7 mean there is a optimal solution. return -1 mean no solution
// Die Rückgabe von 0 bis 7 bedeutet, dass es eine optimale Lösung gibt. Die Rückgabe von -1 bedeutet, dass es keine Lösung gibt
int IK_BestSol(float joint_angle_result [8][6],float cur_joint_angle[6]);

void print_DH_value();

void print_fk_result();

// id for result in radian(0) or degree(1)
void print_IK_result (int id);

// id for result in radian(0) or degree(1)
void print_joint_angle(float joint_angle[6], int id);

void print_IK_matric(float IK_matric [4][4]);


#endif //__RAG2_KINEMATIC_H