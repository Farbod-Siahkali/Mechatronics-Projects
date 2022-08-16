clear all
clc
a = -20;
b = 70;
c = -84;
d = 35;
e = 0;
f = 0;

%% initial and final joint angles
teta_i = [0; pi/2; 0];
theta_f = [-pi/2 ; pi/3 ; -pi/6];

tf = 1;
Ts_M = 0.00169;
Ts = 0.001;
T = 0 : Ts_M : tf;

p=0;

for j = 0 : Ts_M : tf
    p = p + 1;
    t_n = j/tf;
    
    s(p) = a * t_n^ 7 + b * t_n ^ 6 + c * t_n ^ 5 + d * t_n ^ 4;
    s_prime(p) = 7*a*t_n^6 + 6*b*t_n^5 + 5*c*t_n^4 + 4*d*t_n^3;
    s_second(p) = 42*a*t_n^5 + 30*b*t_n^4 + 20*c*t_n^3 + 12*d*t_n^2;
    
    Teta = teta_i + (theta_f - teta_i) * s(p);
    Teta = (Teta - round( Teta /2 / pi ) *2*pi);
    teta_dot = ((theta_f - teta_i) * s_prime(p) / tf);
    teta_ddot = ((theta_f - teta_i) * s_second(p) / tf^2);    
    
    Teta1(p) = Teta(1);
    Teta2(p) = Teta(2);
    Teta3(p) = Teta(3);    
    
    dot_Teta1(p) = teta_dot(1);
    dot_Teta2(p) = teta_dot(2);
    dot_Teta3(p) = teta_dot(3);    
    
    ddot_Teta1(p) = teta_ddot(1);
    ddot_Teta2(p) = teta_ddot(2);
    ddot_Teta3(p) = teta_ddot(3);
end

theta1_timeseries = timeseries(Teta1,T);
theta2_timeseries = timeseries(Teta2,T);
theta3_timeseries = timeseries(Teta3,T);

%Plotting the estimation of Joint 1 Torque
Tau1 = (ddot_Teta2.*(cos(Teta2).*((cos(Teta1).*sin(Teta2))/2000000 - (cos(Teta1).*cos(Teta2))/2000000 + (cos(Teta2).*sin(Teta1))/2000000 + (sin(Teta1).*sin(Teta2))/2000000) + sin(Teta2).*((3*cos(Teta1).*sin(Teta2))/200000 - (cos(Teta1).*cos(Teta2))/2000000 + (3.*cos(Teta2).*sin(Teta1))/200000 + (sin(Teta1).*sin(Teta2))/2000000) + (cos(Teta1).*sin(Teta2) + cos(Teta2).*sin(Teta1)).*(cos(Teta2)/2000000 + (3*sin(Teta2))/200000) - (cos(Teta1).*cos(Teta2) - sin(Teta1).*sin(Teta2)).*(cos(Teta2)/2000000 + sin(Teta2)/2000000) + 1/1000000) + dot_Teta2.*(cos(Teta2).*((cos(Teta1).*cos(Teta2))/2000000 + (cos(Teta1).*sin(Teta2))/2000000 + (cos(Teta2).*sin(Teta1))/2000000 - (sin(Teta1).*sin(Teta2))/2000000) + sin(Teta2).*((3*cos(Teta1).*cos(Teta2))/200000 + (cos(Teta1).*sin(Teta2))/2000000 + (cos(Teta2).*sin(Teta1))/2000000 - (3*sin(Teta1).*sin(Teta2))/200000) + (cos(Teta1).*cos(Teta2) - sin(Teta1).*sin(Teta2)).*(cos(Teta2)/2000000 + (3*sin(Teta2))/200000) + (cos(Teta1).*sin(Teta2) + cos(Teta2).*sin(Teta1)).*(cos(Teta2)/2000000 + sin(Teta2)/2000000)) - (dot_Teta3 + dot_Teta2.*cos(Teta2)).*((dot_Teta1.*cos(Teta1).*cos(Teta2))/2000000 + (dot_Teta1.*cos(Teta1).*sin(Teta2))/2000000 + (dot_Teta1.*cos(Teta2).*sin(Teta1))/2000000 - (dot_Teta1.*sin(Teta1).*sin(Teta2))/2000000) + (dot_Teta1.*cos(Teta1).*cos(Teta2) - dot_Teta1.*sin(Teta1).*sin(Teta2)).*((dot_Teta1.*cos(Teta1).*cos(Teta2))/2000000 + (dot_Teta1.*cos(Teta1).*sin(Teta2))/40000 + (dot_Teta1.*cos(Teta2).*sin(Teta1))/40000 - (dot_Teta1.*sin(Teta1).*sin(Teta2))/2000000) - (dot_Teta1.*cos(Teta1).*cos(Teta2) - dot_Teta1.*sin(Teta1).*sin(Teta2)).*(dot_Teta3/2000000 + (dot_Teta2.*cos(Teta2))/2000000 + (3*dot_Teta2.*sin(Teta2))/200000 - (dot_Teta1.*cos(Teta1).*cos(Teta2))/2000000 + (3*dot_Teta1.*cos(Teta1).*sin(Teta2))/200000 + (3*dot_Teta1.*cos(Teta2).*sin(Teta1))/200000 + (dot_Teta1.*sin(Teta1).*sin(Teta2))/2000000) - (dot_Teta1.*cos(Teta1).*sin(Teta2) + dot_Teta1.*cos(Teta2).*sin(Teta1)).*(dot_Teta3/2000000 + (dot_Teta2.*cos(Teta2))/2000000 + (dot_Teta2.*sin(Teta2))/2000000 - (dot_Teta1.*cos(Teta1).*cos(Teta2))/40000 + (dot_Teta1.*cos(Teta1).*sin(Teta2))/2000000 + (dot_Teta1.*cos(Teta2).*sin(Teta1))/2000000 + (dot_Teta1.*sin(Teta1).*sin(Teta2))/40000) + dot_Teta1.*(2*(cos(Teta1).*sin(Teta2) + cos(Teta2).*sin(Teta1)).*((cos(Teta1).*sin(Teta2))/2000000 - (cos(Teta1).*cos(Teta2))/40000 + (cos(Teta2).*sin(Teta1))/2000000 + (sin(Teta1).*sin(Teta2))/40000) - 2*(cos(Teta1).*cos(Teta2) - sin(Teta1).*sin(Teta2)).*((cos(Teta1).*cos(Teta2))/2000000 + (cos(Teta1).*sin(Teta2))/40000 + (cos(Teta2).*sin(Teta1))/40000 - (sin(Teta1).*sin(Teta2))/2000000) + 2*(cos(Teta1).*sin(Teta2) + cos(Teta2).*sin(Teta1)).*((3*cos(Teta1).*cos(Teta2))/200000 + (cos(Teta1).*sin(Teta2))/2000000 + (cos(Teta2).*sin(Teta1))/2000000 - (3*sin(Teta1).*sin(Teta2))/200000) + 2*(cos(Teta1).*cos(Teta2) - sin(Teta1).*sin(Teta2)).*((3*cos(Teta1).*sin(Teta2))/200000 - (cos(Teta1).*cos(Teta2))/2000000 + (3*cos(Teta2).*sin(Teta1))/200000 + (sin(Teta1).*sin(Teta2))/2000000)) + ddot_Teta3.*((cos(Teta1).*sin(Teta2))/1000000 - (cos(Teta1).*cos(Teta2))/1000000 + (cos(Teta2).*sin(Teta1))/1000000 + (sin(Teta1).*sin(Teta2))/1000000) + dot_Teta3.*((cos(Teta1).*cos(Teta2))/1000000 + (cos(Teta1).*sin(Teta2))/1000000 + (cos(Teta2).*sin(Teta1))/1000000 - (sin(Teta1).*sin(Teta2))/1000000) + ddot_Teta1.*(2*(cos(Teta1).*sin(Teta2) + cos(Teta2).*sin(Teta1)).*((3*cos(Teta1).*sin(Teta2))/200000 - (cos(Teta1).*cos(Teta2))/2000000 + (3*cos(Teta2).*sin(Teta1))/200000 + (sin(Teta1).*sin(Teta2))/2000000) - 2*(cos(Teta1).*cos(Teta2) - sin(Teta1).*sin(Teta2)).*((cos(Teta1).*sin(Teta2))/2000000 - (cos(Teta1).*cos(Teta2))/40000 + (cos(Teta2).*sin(Teta1))/2000000 + (sin(Teta1).*sin(Teta2))/40000) + 89/200000) - (dot_Teta2.*sin(Teta2) + dot_Teta1.*cos(Teta1).*sin(Teta2) + dot_Teta1.*cos(Teta2).*sin(Teta1)).*((3*dot_Teta1.*cos(Teta1).*cos(Teta2))/200000 + (dot_Teta1.*cos(Teta1).*sin(Teta2))/2000000 + (dot_Teta1.*cos(Teta2).*sin(Teta1))/2000000 - (3*dot_Teta1.*sin(Teta1).*sin(Teta2))/200000))*100/3;
figure(1)
plot(Tau1,'r')
title('Torque Joint 1')
ylabel('Torque (n.m)')
grid on

%Plotting the estimation of Joint 2 Torque
Tau2 = ((9*cos(Teta2 + Teta3))/20 + (27*cos(Teta2))/20 + ddot_Teta1.*(cos(Teta2).*((cos(Teta1).*sin(Teta2))/2000000 - (cos(Teta1).*cos(Teta2))/2000000 + (cos(Teta2).*sin(Teta1))/2000000 + (sin(Teta1).*sin(Teta2))/2000000) + sin(Teta2).*((3*cos(Teta1).*sin(Teta2))/200000 - (cos(Teta1).*cos(Teta2))/2000000 + (3*cos(Teta2).*sin(Teta1))/200000 + (sin(Teta1).*sin(Teta2))/2000000) + (cos(Teta1).*sin(Teta2) + cos(Teta2).*sin(Teta1)).*(cos(Teta2)/2000000 + (3*sin(Teta2))/200000) - (cos(Teta1).*cos(Teta2) - sin(Teta1).*sin(Teta2)).*(cos(Teta2)/2000000 + sin(Teta2)/2000000) + 1/1000000) + dot_Teta1.*(cos(Teta2).*((cos(Teta1).*cos(Teta2))/2000000 + (cos(Teta1).*sin(Teta2))/2000000 + (cos(Teta2).*sin(Teta1))/2000000 - (sin(Teta1).*sin(Teta2))/2000000) + sin(Teta2).*((3*cos(Teta1).*cos(Teta2))/200000 + (cos(Teta1).*sin(Teta2))/2000000 + (cos(Teta2).*sin(Teta1))/2000000 - (3*sin(Teta1).*sin(Teta2))/200000) + (cos(Teta1).*cos(Teta2) - sin(Teta1).*sin(Teta2)).*(cos(Teta2)/2000000 + (3*sin(Teta2))/200000) + (cos(Teta1).*sin(Teta2) + cos(Teta2).*sin(Teta1)).*(cos(Teta2)/2000000 + sin(Teta2)/2000000)) - (dot_Teta2.*sin(Teta2) + dot_Teta1.*cos(Teta1).*sin(Teta2) + dot_Teta1.*cos(Teta2).*sin(Teta1)).*((3*dot_Teta2.*cos(Teta2))/200000 - (dot_Teta2.*sin(Teta2))/2000000 + (3*dot_Teta1.*cos(Teta1).*cos(Teta2))/200000 + (dot_Teta1.*cos(Teta1).*sin(Teta2))/2000000 + (dot_Teta1.*cos(Teta2).*sin(Teta1))/2000000 - (3*dot_Teta1.*sin(Teta1).*sin(Teta2))/200000) - (dot_Teta1.*cos(Teta1).*sin(Teta2) + dot_Teta1.*cos(Teta2).*sin(Teta1)).*(dot_Teta3/2000000 + (dot_Teta2.*cos(Teta2))/2000000 + (dot_Teta2.*sin(Teta2))/2000000 - (dot_Teta1.*cos(Teta1).*cos(Teta2))/40000 + (dot_Teta1.*cos(Teta1).*sin(Teta2))/2000000 + (dot_Teta1.*cos(Teta2).*sin(Teta1))/2000000 + (dot_Teta1.*sin(Teta1).*sin(Teta2))/40000) + ddot_Teta2.*(2*cos(Teta2).*(cos(Teta2)/50000 + sin(Teta2)/2000000) + 2*sin(Teta2).*(cos(Teta2)/2000000 + (3*sin(Teta2))/200000) + 103/100000) - (dot_Teta3 + dot_Teta2.*cos(Teta2)).*((dot_Teta2.*cos(Teta2))/2000000 - (dot_Teta2.*sin(Teta2))/50000 + (dot_Teta1.*cos(Teta1).*cos(Teta2))/2000000 + (dot_Teta1.*cos(Teta1).*sin(Teta2))/2000000 + (dot_Teta1.*cos(Teta2).*sin(Teta1))/2000000 - (dot_Teta1.*sin(Teta1).*sin(Teta2))/2000000) + (dot_Teta1.*cos(Teta1).*cos(Teta2) - dot_Teta1.*sin(Teta1).*sin(Teta2)).*((dot_Teta2.*cos(Teta2))/2000000 - (dot_Teta2.*sin(Teta2))/2000000 + (dot_Teta1.*cos(Teta1).*cos(Teta2))/2000000 + (dot_Teta1.*cos(Teta1).*sin(Teta2))/40000 + (dot_Teta1.*cos(Teta2).*sin(Teta1))/40000 - (dot_Teta1.*sin(Teta1).*sin(Teta2))/2000000) + dot_Teta2.*(2*cos(Teta2).*(cos(Teta2)/2000000 - sin(Teta2)/50000) + 2*cos(Teta2).*(cos(Teta2)/2000000 + (3*sin(Teta2))/200000) - 2*sin(Teta2).*(cos(Teta2)/50000 + sin(Teta2)/2000000) + 2*sin(Teta2).*((3*cos(Teta2))/200000 - sin(Teta2)/2000000)) + ddot_Teta3.*(cos(Teta2)/25000 + sin(Teta2)/1000000) + dot_Teta3.*(cos(Teta2)/1000000 - sin(Teta2)/25000) - (dot_Teta2.*cos(Teta2) + dot_Teta1.*cos(Teta1).*cos(Teta2) - dot_Teta1.*sin(Teta1).*sin(Teta2)).*(dot_Teta3/2000000 + (dot_Teta2.*cos(Teta2))/2000000 + (3*dot_Teta2.*sin(Teta2))/200000 - (dot_Teta1.*cos(Teta1).*cos(Teta2))/2000000 + (3*dot_Teta1.*cos(Teta1).*sin(Teta2))/200000 + (3*dot_Teta1.*cos(Teta2).*sin(Teta1))/200000 + (dot_Teta1.*sin(Teta1).*sin(Teta2))/2000000) + dot_Teta2.*sin(Teta2).*(dot_Teta3/50000 + (dot_Teta2.*cos(Teta2))/50000 + (dot_Teta2.*sin(Teta2))/2000000 - (dot_Teta1.*cos(Teta1).*cos(Teta2))/2000000 + (dot_Teta1.*cos(Teta1).*sin(Teta2))/2000000 + (dot_Teta1.*cos(Teta2).*sin(Teta1))/2000000 + (dot_Teta1.*sin(Teta1).*sin(Teta2))/2000000))*(-1)+0.5;
figure(2)
plot(Tau2,'r')
title('Torque Joint 2')
ylabel('Torque (n.m)')
grid on

%Plotting the estimation of Joint 3 Torque
Tau3 = (61*ddot_Teta3)/25000 + (9*cos(Teta2 + Teta3))/20 + ddot_Teta1.*((cos(Teta1).*sin(Teta2))/1000000 - (cos(Teta1).*cos(Teta2))/1000000 + (cos(Teta2).*sin(Teta1))/1000000 + (sin(Teta1).*sin(Teta2))/1000000) + dot_Teta1.*((cos(Teta1).*cos(Teta2))/1000000 + (cos(Teta1).*sin(Teta2))/1000000 + (cos(Teta2).*sin(Teta1))/1000000 - (sin(Teta1).*sin(Teta2))/1000000) + ddot_Teta2.*(cos(Teta2)/25000 + sin(Teta2)/1000000) + dot_Teta2.*(cos(Teta2)/1000000 - sin(Teta2)/25000)-0.3;
figure(3)
plot(Tau3,'r')
title('Torque Joint 3')
ylabel('Torque (n.m)')
grid on