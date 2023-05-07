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
// it should be in radian
float joint_angle[6]={(0*PI)/180, (90*PI)/180,(20*PI)/180,(20*PI)/180,(10*PI)/180,(20*PI)/180};


// Parameters for DH convention (i-1 th ahlpe, i-1 th a, i th d, i th theta)
// this martic give the info of robot structure and state 
float DH_value [6][4]={ {0,0,0,joint_angle[0]},
                        {-0.5*PI,0,0,joint_angle[1]},
                        {0,250,0,joint_angle[2]},
                        {-0.5*PI,0,250,joint_angle[3]},
                        {0.5*PI,0,0,joint_angle[4]},
                        {-0.5*PI,0,0,joint_angle[5]}
                        };

// Result of forward kinematic. (FK output)
//it give a martic that give info of the wrist coordinate and orientation
float fk_matric_result [4][4];

// wrist coordinate martic for inverse kinematic (IK input)
float IK_matric [4][4]={{0.422163,      -0.757678,      0.497698,       191.8  },
                        {-0.832499,     -0.541327,      -0.117944,      -69.8094       },
                        {0.358781,      -0.364541,      -0.859294,      54.6908        },
                        {0,       0,       0,       1}};

// Result of inverse kinematic. (IK output)
// it give <=8 different result of the joint value of a given state
float joint_angle_result [8][6];

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


void print_DH_value();

void print_fk_result();

// id for result in radian(0) or degree(1)
void print_IK_result (int id);

// id for result in radian(0) or degree(1)
void print_joint_angle(float joint_angle[6], int id);

void print_IK_matric(float IK_matric [4][4]);

#endif //__RAG2_KINEMATIC_H