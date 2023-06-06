/**
 * @file  RAG2_kinematic.hpp
 * @author kaming (kmlamal@connect.ust.hk)
 * @brief forward and inverse kinematics for a RAG2 robotic arm
 * @version 0.1
 * @date 2022-6-5
 *
 * 
 */

#pragma once

#include <math.h>
#include <iostream>
#include <cstring> 
// cout and endl just for testing. need to deleted afterward
using std::cout;
using std::endl;

#define PI 3.14159265


class RAG2_kinematic
{
private:
    // current Joint angle. should keep update
    float cur_angle[6];

    // Parameters for DH convention (i-1 th ahlpe, i-1 th a, i th d, i th theta)
    // this martic give the info of robot structure and state 
    // the joint 7 is just a translation from wrist coord to tools coord
    float DH_value [7][4];

    // Matrix to store the transformation matrix for each link. for Forward kinematic
    float fk_matric [7][4][4];

    // Result of forward kinematic. (FK output)
    //martic that give info of the tools coordinate and orientation
    float fk_matric_result [4][4];

    // inverse translation matrix from tool coord to wrist coord 
    const float inv_Joint_7_matrix [4][4]={ {   1,  0,  0,  0},
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

    // the return value of IK_general function 
    // valid = 1 mean that the there is a solution. otherwise
    // joint_state is the return solution of six joint anlge
    short valid;
    float best_joint_state[6];

    // previous valid result from IK
    float prev_joint_state[6];




public:
    // the init_angle need to convert from real world robot angle to virtual manuel
    RAG2_kinematic(float link1, float link2, float link3 ,float init_angle[6]);

    ~RAG2_kinematic();
    
    void set_cur_angle(float cur_angle[6]){ memcpy (this->cur_angle, cur_angle,24); }

    short return_valid(){return this->valid;}

    float* return_bt_joint_state(){return best_joint_state;}
    
    float (* return_fk_result())[4] {return fk_matric_result;}
    
    // 4x4 matric multiply
    void multiply(const float mat1[4][4],const float mat2[4][4],float res[4][4]); 

    // // forward kinematic calculation
    void FK_calculation (float joint_angle[6]);

    // // inverse kinematic calculation
    void IK_calculation(float IK_matric [4][4]);

    // construction of translation_matric for IK input. using the x-y-z fixed angle systems or z-y-x Euler angle systems
    // x-y-z fixed coordinate is mean by first rotate x axis by Rx then y-axis by Ry then z-axis by Rz. IN fixed coord To perform rotation of a coordinate
    // Z-Y-X Euler coordinate is mean by first rotate Z-axis by Rz then y-axis by Ry then X-axis by Rx. IN Rotating coord To perform rotation of a coordinate
    // the last three parameter is is the tranlation from A to B by x-asix x mm then y asix y mm then z asix z mm
    void translation_matric(float IK_matric [4][4], float Rx,float Ry,float Rz,float x,float y,float z);

    // construction of translation_matric for IK input. using the Euler parameter/ Quaternion with Hamilton convention
    // Please reference to the bood and https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
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
    void IK_BestSol();

    // function you should call. Combinated IK solution
    // before using it please first update the current joint state.
    // parameter is Quaternion with Hamilton convention and x,y,z is the translation of the tool coord rwt basic coord
    // if there are sol, the result will store in best_joint_state. if not, previous result of calling this function with store in best_joint_state.
    // parameter valid is the reture of whether is there any valid sol
    // still need to update with the R2V V2R function. it require comfirm of motor init state
    void ik_general (float qx,float qy,float qz,float qw,float x,float y,float z);

};
