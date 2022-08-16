%Forward Kinematics
syms teta1 teta2 teta3
a1=[0;
    0;
    0.21];

a2=[0.21*cos(teta2);
    0.21*sin(teta2);
          0];

a3=[0.18*cos(teta3);
    0.18*sin(teta3);
          0];

Q1=[cos(teta1) 0 sin(teta1);
    sin(teta1) 0 -cos(teta1);
    0          1      0];

Q2=[cos(teta2) -sin(teta2) 0;
    sin(teta2) cos(teta2)  0;
    0             0        1];

Q3=[cos(teta3) 0 sin(teta3);
    sin(teta3) 0 -cos(teta3); 
    0          1       0];

P = a1+Q1*a2+Q1*Q2*a3

simplify(P);

%Dynamic Problem
syms dteta1 dteta2 dteta3
w1_1 = dteta1*[0; 0; 1];
w2_2 = dteta2*[0; 0; 1];
w3_3 = dteta3*[0; 0; 1];

c1_1 = [-0.01; 0.01; 0.2];
c2_2 = [0.01; 0.04; -0.01];
c3_3 = [0.02; 0.06; -0.02];

dc1 = cross(w1_1, c1_1);
dc2 = cross(w2_2, c2_2);
dc3 = cross(w3_3, c3_3);

w1_2=dteta1*transpose(Q1)*[0;0;1];
w2_3=dteta1*transpose(Q1)*transpose(Q2)*[0;0;1]+dteta2*transpose(Q2)*[0;0;1];
w3_4=dteta1*transpose(Q1)*transpose(Q2)*transpose(Q3)*[0;0;1]+dteta2*transpose(Q2)*transpose(Q3)*[0;0;1]+dteta3*transpose(Q3)*[0;0;1];

I1=[20 1 1; 1 25 1; 1 1 50]*10^(-6);
I2=[50 1 1; 1 300 1; 1 1 10]*10^(-6);
I3=[30 1 1; 1 40 1; 1 1 50]*10^(-6);

%Kinetic Calculation
T1 = 0.3*transpose(dc1)*dc1+0.5*transpose(w1_2)*I1*w1_2;
T2 = 0.3*transpose(dc2)*dc2+0.5*transpose(w2_3)*I2*w2_3;
T3 = 0.3*transpose(dc3)*dc3+0.5*transpose(w3_4)*I3*w3_4;

T = T1+T2+T3

M11 = gradient(gradient(T, dteta1), dteta1);
M12 = gradient(gradient(T, dteta2), dteta1);
M13 = gradient(gradient(T, dteta3), dteta1);
M21 = gradient(gradient(T, dteta1), dteta2);
M22 = gradient(gradient(T, dteta2), dteta2);
M23 = gradient(gradient(T, dteta3), dteta2);
M31 = gradient(gradient(T, dteta1), dteta3);
M32 = gradient(gradient(T, dteta2), dteta3);
M33 = gradient(gradient(T, dteta3), dteta3);

M=[M11 M12 M13; 
   M21 M22 M23;
   M31 M32 M33];

diff_M11 = diff(M11);
diff_M12 = diff(M12);
diff_M13 = diff(M13);
diff_M21 = diff(M21);
diff_M22 = diff(M22);
diff_M23 = diff(M23);
diff_M31 = diff(M31);
diff_M32 = diff(M32);
diff_M33 = diff(M33);

diff_M = diff(M);

diff_T1 = gradient(T, teta1);
diff_T2 = gradient(T, teta2);
diff_T3 = gradient(T, teta3);

diff_T = [diff_T1; diff_T2; diff_T3];

%Potential Calculation
V = 1.35*sin(teta2)+0.45*sin(teta2+teta3)

diff_V1 = gradient(V,teta1);
diff_V2 = gradient(V,teta2);
diff_V3 = gradient(V,teta3);

diff_V = [diff_V1; diff_V2; diff_V3];

%Torque Calculation
syms d2teta1 d2teta2 d2teta3

Tau = M*[d2teta1;d2teta2;d2teta3]+diff_M*[dteta1;dteta2;dteta3]-diff_T+diff_V

Tau1 = M11*d2teta1+M12*d2teta2+M13*d2teta3+diff_M11*dteta1+diff_M12*dteta2+diff_M13*dteta3-diff_T1+diff_V1
Tau2 = M21*d2teta1+M22*d2teta2+M23*d2teta3+diff_M21*dteta1+diff_M22*dteta2+diff_M23*dteta3-diff_T2+diff_V2
Tau3 = M31*d2teta1+M32*d2teta2+M33*d2teta3+diff_M31*dteta1+diff_M32*dteta2+diff_M33*dteta3-diff_T3+diff_V3
