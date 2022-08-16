clc
clear all
%% plynomianl 4 5 6 7  
a = -20;
b = 70;
c = -84;
d = 35;
e = 0;
f = 0;

%% initial and final joint angles
theta_i = [0 ; 0 ; 0];
theta_f = [-pi/2 ; pi/3 ; -pi/6];
%%
tf = 1;
Ts_M = 0.0001;
Ts = 0.001;
T = 0 : Ts_M : tf;
%%
p=0;
for j = 0 : Ts_M : tf
    p = p + 1;
    t_n = j/tf;
    
    s(p) = a * t_n^ 7 + b * t_n ^ 6 + c * t_n ^ 5 + d * t_n ^ 4;
    s_prime(p) = 7*a*t_n^6 + 6*b*t_n^5 + 5*c*t_n^4 + 4*d*t_n^3;
    s_second(p) = 42*a*t_n^5 + 30*b*t_n^4 + 20*c*t_n^3 + 12*d*t_n^2;
    
% % %     
    theta = theta_i + (theta_f - theta_i) * s(p);
    theta = (theta - round( theta /2 / pi ) *2*pi);
    theta_dot = ((theta_f - theta_i) * s_prime(p) / tf);
    theta_ddot = ((theta_f - theta_i) * s_second(p) / tf^2);
  
%     
    
    theta1_num(p) = theta(1);
    theta2_num(p) = theta(2);
    theta3_num(p) = theta(3);
    %theta4_num(p) = theta(4);
     
    theta1_dot_num(p) = theta_dot(1);
    theta2_dot_num(p) = theta_dot(2);
    theta3_dot_num(p) = theta_dot(3);
    %theta4_dot_num(p) = theta_dot(4);
     
    theta1_ddot_num(p) = theta_ddot(1);
    theta2_ddot_num(p) = theta_ddot(2);
    theta3_ddot_num(p) = theta_ddot(3);
    %theta4_ddot_num(p) = theta_ddot(4);
    
end
%%

theta1_timeseries = timeseries(theta1_num,T);
theta2_timeseries = timeseries(theta2_num,T);
theta3_timeseries = timeseries(theta3_num,T);
%theta4_timeseries = timeseries(theta4_num,T);