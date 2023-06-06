/**
 * @file  RAG2_kinematic.cpp
 * @author kaming (kmlamal@connect.ust.hk)
 * @brief forward and inverse kinematics for a RAG2 robotic arm
 * @version 0.1
 * @date 2022-6-5
 *
 * 
 */

#include "RAG2_kinematic.hpp"

RAG2_kinematic::RAG2_kinematic(float link1, float link2, float link3,float init_angle[6])    
    : DH_value       {  {0,0,0,init_angle[0]},
                        {-0.5*PI,0,0,init_angle[1]},
                        {0,link1,0,init_angle[2]},
                        {-0.5*PI,0,link2,init_angle[3]},
                        {0.5*PI,0,0,init_angle[4]},
                        {-0.5*PI,0,0,init_angle[5]},
                        {0,0,link3,0}}
{
    //////////////////function member init////////////////////////////////////
    memcpy(cur_angle,init_angle,sizeof(cur_angle));
    memcpy(prev_joint_state,init_angle,sizeof( prev_joint_state));

    //////////////////init forward kinematic matric///////////////////////////
    for (int i=0; i<7;i++){
        fk_matric [i][0][0] = cos( DH_value[i][3]  );
        fk_matric [i][0][1] = - sin( DH_value[i][3]  );
        fk_matric [i][0][2] =0;
        fk_matric [i][0][3] = DH_value[i][1];

        fk_matric [i][1][0] = sin(DH_value[i][3] ) * cos( DH_value[i][0] );
        fk_matric [i][1][1] = cos(DH_value[i][3] ) * cos( DH_value[i][0] );
        fk_matric [i][1][2] = -sin(DH_value[i][0]);
        fk_matric [i][1][3] = -sin(DH_value[i][0]) *  DH_value[i][2];

        fk_matric [i][2][0] = sin(DH_value[i][3] ) * sin(DH_value[i][0] );
        fk_matric [i][2][1] = cos(DH_value[i][3] ) * sin( DH_value[i][0] );
        fk_matric [i][2][2] = cos(DH_value[i][0]);
        fk_matric [i][2][3] = cos(DH_value[i][0]) *  DH_value[i][2];

        fk_matric [i][3][0] = 0;
        fk_matric [i][3][1] = 0;
        fk_matric [i][3][2] = 0;
        fk_matric [i][3][3] = 1;
    }

    // init for the end joint translation joint_7 which is a fix joint angle 0
        fk_matric [6][0][0] = 1;
        fk_matric [6][0][1] = 0;
        fk_matric [6][1][0] = 0;
        fk_matric [6][1][1] = 1;
        fk_matric [6][2][0] = 0;
        fk_matric [6][2][1] = 0;


}

RAG2_kinematic::~RAG2_kinematic()
{
}

void RAG2_kinematic::multiply(const float mat1[4][4],
              const float mat2[4][4],
              float res[4][4])
{
    int i, j, k;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            res[i][j] = 0;
            for (k = 0; k < 4; k++)
                res[i][j] += mat1[i][k] * mat2[k][j];
        }
    }
}

void RAG2_kinematic::FK_calculation (float joint_angle[6])
{
    // clear previous result
    for (int i=0; i<4;i++){
        for (int j=0; j<4;j++){
            this->fk_matric_result [i][j] =0;
    }}

    //init and update value into the link martic
    for (int i=0; i<6;i++)
    {
        fk_matric [i][0][0] = cos( joint_angle[i]  );
        fk_matric [i][0][1] = - sin( joint_angle[i]  );
        fk_matric [i][1][0] = sin(joint_angle[i] ) * cos( DH_value[i][0] );
        fk_matric [i][1][1] = cos(joint_angle[i] ) * cos( DH_value[i][0] );
        fk_matric [i][2][0] = sin(joint_angle[i] ) * sin( DH_value[i][0] );
        fk_matric [i][2][1] = cos(joint_angle[i] ) * sin( DH_value[i][0] );
    
    }
    // start from the end joint 7
    float temp_result [4][4];
    memcpy (temp_result, fk_matric[6], sizeof(fk_matric_result));


    // metric calculation for forward kinematic
    for (int i =5; i>=0 ; i--)
    {
        multiply(fk_matric[i],temp_result,fk_matric_result);
        memcpy(temp_result,fk_matric_result,sizeof(fk_matric_result));
    } 

}


void RAG2_kinematic::IK_calculation(float IK_matric [4][4])
{
/////////////////////// pre-process //////////////////////////////
    // clear previous result
    for (int i=0; i<8;i++){
        for (int j=0; j<6;j++){
            joint_angle_result[i][j] =0;
    }}

    // translate from tool coord to wrist coord
    float temp_IK [4][4];
    memcpy (temp_IK, IK_matric,64); 
    multiply(temp_IK,inv_Joint_7_matrix,IK_matric);


    // init fixed parameter for later use 
    float a2 = DH_value[2][1];
    float d4 = DH_value[3][2];
    float px = IK_matric[0][3]; 
    float py = IK_matric[1][3]; 
    float pz = IK_matric[2][3];
    
    float square = px*px + py*py;
    float K = (px*px+py*py+pz*pz
                -a2*a2-d4*d4)/(2*a2);
/////////////////////// joint 1//////////////////////////////
    for (int joint_1=0;joint_1<4;joint_1++){
        joint_angle_result[joint_1][0] = atan2(py,px)
                                   -atan2(0,sqrt(square));
    }
    for (int joint_1=4;joint_1<8;joint_1++){
        joint_angle_result[joint_1][0] = atan2(py,px)
                                    -atan2(0,-sqrt(square));
    }
    
/////////////////////// joint 3 //////////////////////////////
    for (int joint_3=0;joint_3<2;joint_3++){
        joint_angle_result[joint_3][2] = atan2(0,d4)-atan2(K,sqrt(d4*d4-K*K));
        joint_angle_result[joint_3+4][2] = atan2(0,d4)-atan2(K,sqrt(d4*d4-K*K));
    }
    for (int joint_3=0;joint_3<2;joint_3++){
        joint_angle_result[joint_3+2][2] = atan2(0,d4)-atan2(K,-sqrt(d4*d4-K*K));
        joint_angle_result[joint_3+6][2] = atan2(0,d4)-atan2(K,-sqrt(d4*d4-K*K));
    }

/////////////////////// joint 2 //////////////////////////////
    for (int i=0; i<8 ;i +=2){
        float c1 = cos (joint_angle_result[0+i][0]);
        float s1 = sin (joint_angle_result[0+i][0]);
        float c3 = cos (joint_angle_result[0+i][2]);
        float s3 = sin (joint_angle_result[0+i][2]);
        float theta23 = atan2(-(a2*c3)*pz + ((c1*px)+(s1*py))*(-d4+(a2*s3)),((a2*s3-d4)*pz)+(a2*c3)*((c1*px)+(s1*py)));
        joint_angle_result[0+i][1] = theta23 - joint_angle_result[0+i][2];
        joint_angle_result[1+i][1] = theta23 - joint_angle_result[0+i][2];
    }

/////////////////////// joint_456 //////////////////////////////
    for (int i=0; i<8 ;i +=2){
        float r13   = IK_matric[0][2];
        float s1    = sin(joint_angle_result[0+i][0]);
        float r23   = IK_matric[1][2];
        float c1    = cos(joint_angle_result[0+i][0]);
        float s3    = sin(joint_angle_result[0+i][2]);
        float c3    = cos(joint_angle_result[0+i][2]);
        float c23   = (((a2*s3-d4)*pz)+(a2*c3)*((c1*px)+(s1*py)))  / ((pz*pz)+((c1*px+s1*py)*(c1*px+s1*py)));
        float r33   = IK_matric[2][2];
        float s23   = (-(a2*c3)*pz + ((c1*px)+(s1*py))*(-d4+(a2*s3))) / ((pz*pz)+((c1*px+s1*py)*(c1*px+s1*py)));
        // If the angle of joint 5 is zero, then joint 4 and joint 6 are collinear. The angle of joint 4 can be chosen arbitrarily. 
        // this if case is to detect is it colinear. And i arbitrarily chose the angle 0 as joint4 
        // by chosing 0.01. if the angle of joint 5 is smaller than 0.8 degree then joint 4 is zero (0.8 is depand of differnt computer) 
        if (abs((-r13*s1)+(r23*c1))<0.01 && abs((-r13*c1*c23)-(r23*s1*c23)+(r33*s23))<0.01 ){
             joint_angle_result[0+i][3] =0;
        }else{
            joint_angle_result[0+i][3] = atan2( (-r13*s1)+(r23*c1),(-r13*c1*c23)-(r23*s1*c23)+(r33*s23));

        }

        float s4    = sin(joint_angle_result[0+i][3]);
        float c4    = cos(joint_angle_result[0+i][3]);
        float s5    = -r13*(c1*c23*c4+s1*s4) - r23*(s1*c23*c4-c1*s4)+ r33*(s23*c4);
        float c5    = r13*(-c1*s23) + r23*(-s1*s23) + r33*(-c23);
        joint_angle_result[0+i][4] = atan2(s5,c5);

        float r11   = IK_matric[0][0];
        float r21   = IK_matric[1][0];
        float r31   = IK_matric[2][0];
        float s6    = -r11*(c1*c23*s4-s1*c4) - r21*(s1*c23*s4+c1*c4) + r31*(s23*s4);
        float c6    = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5) + r21*((s1*c23*c4-c1*s4)*c5 - s1*s23*s5) - r31*(s23*c4*c5+c23*s5);
        joint_angle_result[0+i][5] = atan2(s6,c6);

        joint_angle_result[1+i][3] = joint_angle_result[0+i][3]+PI;
        joint_angle_result[1+i][4] = -joint_angle_result[0+i][4];
        joint_angle_result[1+i][5] = joint_angle_result[0+i][5]+PI;
    }
}

void RAG2_kinematic::translation_matric(float IK_matric [4][4], float Rx,float Ry,float Rz,float x,float y,float z){
    IK_matric[0][0]= cos(Rz)*cos(Ry);
    IK_matric[0][1]= cos(Rz)*sin(Ry)*sin(Rx)-sin(Rz)*cos(Rx);
    IK_matric[0][2]= cos(Rz)*sin(Ry)*cos(Rx)+sin(Rz)*sin(Rx);
    IK_matric[0][3]= x;

    IK_matric[1][0]= sin(Rz)*cos(Ry);
    IK_matric[1][1]= sin(Rz)*sin(Ry)*sin(Rx)+cos(Rz)*cos(Rx);
    IK_matric[1][2]= sin(Rz)*sin(Ry)*cos(Rx)-cos(Rz)*sin(Rx);
    IK_matric[1][3]= y;

    IK_matric[2][0]= -sin(Ry);
    IK_matric[2][1]= cos(Ry)*sin(Rx);
    IK_matric[2][2]= cos(Ry)*cos(Rx);
    IK_matric[2][3]= z;

    IK_matric[3][0]= 0;
    IK_matric[3][1]= 0;
    IK_matric[3][2]= 0;
    IK_matric[3][3]= 1;

}

void RAG2_kinematic::translation_matric(float IK_matric [4][4], float qx,float qy,float qz,float qw,float x,float y,float z){
    IK_matric[0][0]= 1-2*qy*qy - 2*qz*qz;
    IK_matric[0][1]= 2*qx*qy - 2*qz*qw;
    IK_matric[0][2]= 2*qx*qz + 2*qy*qw;
    IK_matric[0][3]= x;

    IK_matric[1][0]= 2*qx*qy + 2*qz*qw;
    IK_matric[1][1]= 1-2*qx*qx - 2*qz*qz;
    IK_matric[1][2]= 2*qy*qz - 2*qx*qw;
    IK_matric[1][3]= y;

    IK_matric[2][0]= 2*qx*qz - 2*qy*qw;
    IK_matric[2][1]= 2*qy*qz + 2*qx*qw;
    IK_matric[2][2]= 1-2*qx*qx - 2*qy*qy;
    IK_matric[2][3]= z;

    IK_matric[3][0]= 0;
    IK_matric[3][1]= 0;
    IK_matric[3][2]= 0;
    IK_matric[3][3]= 1;

}

int RAG2_kinematic::reaching_space(float IK_matric [4][4]){
    float r = sqrt (IK_matric[0][3]*IK_matric[0][3] + IK_matric[1][3]*IK_matric[1][3] + IK_matric[2][3]*IK_matric[2][3]);
    //Dextruos Workspace r <= 350mm. in this workspace the solution give all solution
    //Reachable workspace r <= 650mm in this workapace it give < 8 solution
    //Unreachable r > 650mm. in this workapace well you know 
    if (r <= 350){
        return 0;
    }else if(r <= 650){
        return 1;
    }else {
        return 2;
    }

}

void RAG2_kinematic::V2R_transform (float virtual_angle[6],float real_angle[6])
{   
    // this part need to be confirm from LGY
    real_angle[0] = virtual_angle[0]; // need to be confirm
    real_angle[1] = -virtual_angle[1]; 
    real_angle[2] = virtual_angle[2]+1.25*PI; 
    real_angle[3] = virtual_angle[3];
    real_angle[4] = -virtual_angle[4]+0.75*PI;
    real_angle[5] = virtual_angle[5];

}

void RAG2_kinematic::R2V_transform (float virtual_angle[6],float real_angle[6])
{   
    // this part need to be confirm from LGY
    virtual_angle[0] = real_angle[0]; // need to be confirm
    virtual_angle[1] = -real_angle[1]; 
    virtual_angle[2] = real_angle[2]-1.25*PI; 
    virtual_angle[3] = real_angle[3];
    virtual_angle[4] = -real_angle[4]-0.75*PI;
    virtual_angle[5] = real_angle[5];

}

void RAG2_kinematic::IK_BestSol()
{

    float lowest_dist=100000;
    this->optimal_index = -1;

    //exclude impossible result 
    for (int i =0; i<8;i++){
        valid_result[i]=true;
        for (int j=0;j<6;j++){
            // exclude NaN result
            if (isnan(joint_angle_result[i][j])){
                valid_result[i]=false;
                break;
            }

            // enforce result between -PI to +PI
            if (joint_angle_result[i][j]>PI){
                joint_angle_result[i][j] -= 2*PI;
            }
            else if (joint_angle_result[i][j]< -PI){
                joint_angle_result[i][j] += 2*PI;
            }
            
            // exclude those out of angle limit
            if (j==0 && (joint_angle_result[i][j]>0.94444*PI || joint_angle_result[i][j]<-0.94444*PI)){
                valid_result[i]=false;
                break;
            }else if (j==1 && (joint_angle_result[i][j]>0 || joint_angle_result[i][j]<-0.75*PI)){
                valid_result[i]=false;
                break;

            }else if (j==2 && (joint_angle_result[i][j]>0.25*PI || joint_angle_result[i][j]<-1.75*PI)){
                valid_result[i]=false;
                break;

            }else if (j==3 && (joint_angle_result[i][j]>0.75*PI || joint_angle_result[i][j]<-0.75*PI)){
                valid_result[i]=false;
                break;

            }else if (j==4 && (joint_angle_result[i][j]>0.75*PI || joint_angle_result[i][j]<-0.75*PI)){
                valid_result[i]=false;
                break;

            }else if (j==5 && (joint_angle_result[i][j]>0.75*PI || joint_angle_result[i][j]<-0.75*PI)){
                valid_result[i]=false;
                break;
            }
            
        }
    }

    // compute the lowest travel distance
    for (int i=0;i<8;i++){
        if (valid_result[i]==true){
            float temp_dist=0;
            for (int j=0;j<6;j++){
                // closest distance calulate of target and current angle
                float cur_dis = abs(joint_angle_result[i][j]-cur_angle[j]);
                if (cur_dis>PI){
                    cur_dis = abs(cur_dis-2*PI);
                }
                // weighted distance summation of target index i (weight = (5-j)*(5-j) ) this can be adjust
                // in here weighted is quadratic in order to reduce the movement of first few joint angle
                temp_dist += (5-j)*(5-j) * cur_dis;
            }
            if (temp_dist<lowest_dist){
                
                lowest_dist = temp_dist;
                optimal_index =i;
            }
        }
    }

    
}

void RAG2_kinematic::ik_general (float qx,float qy,float qz,float qw,float x,float y,float z)
{
    // construction of target translation matrix
    float IK_matric [4][4];
    translation_matric(IK_matric ,qx, qy, qz, qw, x, y, z);

    // exclude if it is out of operation workspace
    if(reaching_space(IK_matric) == 2){  
        this->valid = 0;
        memcpy(best_joint_state,prev_joint_state,sizeof(prev_joint_state));
        return;
    }

    // calculate best IK reuslt for all six joint
    this->valid = 1;
    IK_calculation(IK_matric);
    IK_BestSol();

    // exclude if there isnt any valid solution
    if(optimal_index == -1){  
        valid = 0;
        memcpy(best_joint_state,prev_joint_state,sizeof(prev_joint_state));
        return;
    }
    memcpy(prev_joint_state,joint_angle_result[optimal_index],sizeof(prev_joint_state));
    memcpy(best_joint_state,joint_angle_result[optimal_index],sizeof(best_joint_state));



}

int main (){
    float cur_angle[6]={(-92*PI)/180, (-60*PI)/180,(-20*PI)/180,(50*PI)/180,(-80*PI)/180,(20*PI)/180};
    float joint_angle[6]={(90*PI)/180, (-70*PI)/180,(-20*PI)/180,(50*PI)/180,(-170*PI)/180,(20*PI)/180};
    

    RAG2_kinematic RAG2(250,250,150,cur_angle);
    RAG2.set_cur_angle(cur_angle);

    RAG2.FK_calculation(joint_angle);
    float (*a)[4] = RAG2.return_fk_result();

    RAG2.ik_general(-0.3676381, 0.2506608, -0.5720545, 0.689032, 114.907, 428.597, 196.609);// valid result
    RAG2.ik_general( -0.2176648,0.6299306,-0.1469676, 0.7308967 ,-187.784,19.9533,251.666);// invalid result




    cout << "--------------------------------"<<endl;
    cout << "IK_matric:"<<endl;
    for (int j=0; j<4;j++){
        for (int k=0; k<4;k++){
            cout << a[j][k] << '\t';
        }
        cout <<endl;
    }

    cout << "--------------------------------"<<endl;
    cout << "valid: " << RAG2.return_valid() <<endl;;
    cout << "--------------------------------"<<endl;

    cout << "IK_result:"<<endl;
    float*jo = RAG2.return_bt_joint_state();
    for (int k=0; k<6;k++){
            cout <<jo[k] * 180 / PI<< '\t';
    }
    cout<<endl;
    
    cout << "--------------------------------"<<endl;
}