#include "RAG2_kinematic.h"



// Matrix to store the transformation matrix for each link. for Forward kinematic
float fk_matric [6][4][4];


void multiply(float mat1[4][4],
              float mat2[4][4],
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

void reset_fk_matric (){
    //  clear previous link matric 
    for (int i=0; i<6;i++){
        for (int j=0; j<4;j++){
            for (int k=0; k<4;k++){
                fk_matric[i][j][k] = 0;
    }}}

    // init value 
    for (int i=0; i<6;i++){
        fk_matric [i][0][0] = cos( DH_value[i][3]  );
        fk_matric [i][0][1] = - sin( DH_value[i][3]  );
        fk_matric [i][0][3] = DH_value[i][1];

        fk_matric [i][1][0] = sin(DH_value[i][3] ) * cos( DH_value[i][0] );
        fk_matric [i][1][1] = cos(DH_value[i][3] ) * cos( DH_value[i][0] );
        fk_matric [i][1][2] = -sin(DH_value[i][0]);
        fk_matric [i][1][3] = -sin(DH_value[i][0]) *  DH_value[i][2];

        fk_matric [i][2][0] = sin(DH_value[i][3] ) * sin(DH_value[i][0] );
        fk_matric [i][2][1] = cos(DH_value[i][3] ) * sin( DH_value[i][0] );
        fk_matric [i][2][2] = cos(DH_value[i][0]);
        fk_matric [i][2][3] = cos(DH_value[i][0]) *  DH_value[i][2];

        fk_matric [i][3][3] = 1;
    }

    // clear previous result
    for (int i=0; i<4;i++){
        for (int j=0; j<4;j++){
            fk_matric_result [i][j] =0;
    }}
    //cout << " FK_martic reset"<<endl;

}


void reset_ik_matric ()
{
    // clear previous result
    for (int i=0; i<8;i++){
        for (int j=0; j<6;j++){
            joint_angle_result[i][j] =0;
    }}
    //cout << " IK_martic reset"<<endl;
}

void FK_calculation (float joint_angle[6]){
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
    float temp_result [4][4];
    memcpy (temp_result, fk_matric[5], sizeof(fk_matric_result));


    // metric calculation for forward kinematic
    for (int i =4; i>=0 ; i--)
    {
        multiply(fk_matric[i],temp_result,fk_matric_result);
        memcpy(temp_result,fk_matric_result,sizeof(fk_matric_result));
    } 

    //putting matric for Inverse kinematic testing (testing)
    memcpy (IK_matric, fk_matric_result, sizeof(fk_matric_result));
}


void IK_calculation(float IK_matric [4][4])
{
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

/////////////////////// joint_456-1 //////////////////////////////
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
        joint_angle_result[0+i][3] = atan2( (-r13*s1)+(r23*c1),(-r13*c1*c23)-(r23*s1*c23)+(r33*s23));

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

void print_DH_value()
{
    cout << "--------------------------------"<<endl;
    cout << "DH_value:"<<endl;
    for (int i=0; i<6;i++){
        for (int j=0;j<4;j++){
            cout << DH_value[i][j] << "\t";
        }
        cout<<endl;
    }
    cout << "--------------------------------"<<endl;

}

void print_fk_result()
{ 
    cout << "--------------------------------"<<endl;
    cout << "fk_result:    "<<endl;
    for (int j=0; j<4;j++){
        // cout<< "{";
        for (int k=0; k<4;k++){
            cout << fk_matric_result[j][k]  /*<<","*/<< '\t';
        }
        cout/*<< "},"*/<<endl;
    }
    cout << "--------------------------------"<<endl;
}

// id for result in radian(0) or degree(1)
void print_IK_result (int id)
{
    cout << "--------------------------------"<<endl;
    cout << "IK_result:"<<endl;
    for (int j=0; j<8;j++){
        for (int k=0; k<6;k++){
            if  (id==1){
                cout << joint_angle_result[j][k]*180 /PI << '\t';
            }else {
                cout << joint_angle_result[j][k] << '\t';
            }
        }
        cout<<endl;
    }
    cout << "--------------------------------"<<endl;

}

// id for result in radian(0) or degree(1)
void print_joint_angle(float joint_angle[6], int id)
{
    cout << "--------------------------------"<<endl;
    cout << "joint_value:"<<endl;
    for (int i=0; i<6;i++){
        if  (id==1){
                cout << joint_angle[i]*180 /PI << '\t';
        }else{
                cout << joint_angle[i] << "\t";
        }
        
    }
    cout<<endl;
    cout << "--------------------------------"<<endl;
}

void print_IK_matric(float IK_matric [4][4])
{
    cout << "--------------------------------"<<endl;
    cout << "IK_matric:"<<endl;
    for (int j=0; j<4;j++){
        for (int k=0; k<4;k++){
            cout << IK_matric[j][k] << '\t';
        }
    }
}

// the main is a testing program
int main (){
    //init for every new calculation
    reset_fk_matric ();
    reset_ik_matric ();

    //calculation for FK from template joint_angle
    print_joint_angle(joint_angle,1);
    FK_calculation (joint_angle);
    print_fk_result();

    //calcutation for the result that from previous FK 
    IK_calculation(fk_matric_result);
    print_IK_result (1);

    // choosing one of the result from IK reuslt and test does it give right coordinate matric
    reset_fk_matric ();
    FK_calculation (joint_angle_result[3]);
    print_fk_result();




}




















    


