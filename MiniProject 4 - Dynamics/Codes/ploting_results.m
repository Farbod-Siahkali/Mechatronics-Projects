clc
%%
% tau1
figure(1)
plot(out.tau1_simulink.data,'r')
title('Torque Joint 1')
ylabel('Tau (n.m)')
grid on

%%
%tau2
figure(2)
plot(out.tau2_simulink.data,'r')
title('Torque Joint 2')
ylabel('Tau (n.m)')
grid on

%%
%tau3
figure(3)
plot(out.tau3_simulink.data,'r')
title('Torque Joint 3')
ylabel('Tau (n.m)')
grid on
