#ifndef __EXAMPLE_LINUX_H
#define __EXAMPLE_LINUX_H

int doHardwareScan(xsens::Cmt3 &, CmtDeviceId []);
void doMtSettings(xsens::Cmt3 &, CmtOutputMode &, CmtOutputSettings &, CmtDeviceId []);
void getUserInputs(CmtOutputMode &, CmtOutputSettings &);
void writeHeaders(unsigned long, CmtOutputMode &, CmtOutputSettings &, int&, int&);
int calcScreenOffset(CmtOutputMode &, CmtOutputSettings &, int);


struct rotation_matrix {
	float r1;
	float r2;
	float r3;
	float r4;
	float r5;
	float r6;
	float r7;
	float r8;
	float r9;
	
};

struct rotation_matrix make_rotation_matrix(float roll,float pitch,float yaw)
{
	struct rotation_matrix R;

	float A[3][3] = { {cos(yaw * PI / 180) * cos(pitch * PI / 180), cos(yaw * PI / 180) * sin(pitch * PI / 180) * sin(roll * PI / 180) - sin(yaw * PI / 180) * cos(roll * PI / 180), cos(yaw * PI / 180) * sin(pitch * PI / 180) * cos(roll * PI / 180) + sin(yaw * PI / 180) * sin(roll * PI / 180)},
					  {sin(yaw * PI / 180) * cos(pitch * PI / 180), sin(yaw * PI / 180) * sin(pitch * PI / 180) * sin(roll * PI / 180) + cos(yaw * PI / 180) * cos(roll * PI / 180), sin(yaw * PI / 180) * sin(pitch * PI / 180) * cos(roll * PI / 180) - cos(yaw * PI / 180) * sin(roll * PI / 180)},
					  {-sin(pitch * PI / 180),cos(pitch * PI / 180) * sin(roll * PI / 180),cos(pitch * PI / 180) * cos(roll * PI / 180)} };
	R.r1 = A[0][0];
	R.r2 = A[0][1];
	R.r3 = A[0][2];
	R.r4 = A[1][0];
	R.r5 = A[1][1];
	R.r6 = A[1][2];
	R.r7 = A[2][0];
	R.r8 = A[2][1];
	R.r9 = A[2][2];
	return R;
}

struct rotation_matrix make_R_b_f()
{
	struct rotation_matrix R;


	float R_s_p_stand_T[3][3];
	//float R_s_p_stoop_T[3][3];
	float R_k[3][3];
	float theta;
	float k[3][1];
	float z[3][1] = { {0},
					  {0},
					  {1} };
	float x[3][1];


	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R_s_p_stand_T[i][j] = R_s_p_stand[j][i];
		}
	}
	/*
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R_s_p_stoop_T[i][j] = R_s_p_stoop[j][i];
		}
	}*/

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 1; j++) {
			for (int k = 0; k < 3; k++) {
				R_k[i][j] += R_s_p_stand_T[i][k] * R_s_p_stoop[k][j];
				//R_k[i][j] += R_s_p_stoop_T[i][k] * R_s_p_stand[k][j];
			}
		}
	}

	theta = acos((R_k[0][0]+ R_k[1][1]+ R_k[2][2]-1)/2);

	k[0][0] = 1 / (2 * cos(theta)) * (R_k[2][1] - R_k[1][2]);
	k[1][0] = 1 / (2 * cos(theta)) * (R_k[0][2] - R_k[2][0]);
	k[2][0] = 1 / (2 * cos(theta)) * (R_k[1][0] - R_k[0][1]);

	x[0][0] = k[1][0] * z[2][0] - k[2][0] * z[1][0];
	x[1][0] = k[2][0] * z[0][0] - k[0][0] * z[2][0];
	x[2][0] = k[0][0] * z[1][0] - k[1][0] * z[0][0];

	R.r1 = x[0][0];
	R.r2 = k[0][0];
	R.r3 = z[0][0];
	R.r4 = x[1][0];
	R.r5 = k[1][0];
	R.r6 = z[1][0];
	R.r7 = x[2][0];
	R.r8 = k[2][0];
	R.r9 = z[2][0];

	return R;
}



#endif
