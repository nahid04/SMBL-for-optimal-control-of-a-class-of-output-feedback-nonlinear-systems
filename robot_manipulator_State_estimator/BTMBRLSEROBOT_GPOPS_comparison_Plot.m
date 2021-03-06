%% Paper title "Safety aware model-based reinforcement learning for optimal control of a class of output-feedback nonlinear systems"
%% arXiv:2110.00271
%% Coded by S M Nahid Mahmud, MS Grad Student, Oklahoma State University.
%% nahid.mahmud@okstate.edu
%% This code presents a four-state dynamical system for the parameter estimation
%% The notations are similar to the mentioned paper. 
%% Transformed system denotes the system we have after applying the barrier transformation to the original system.
%% In this document, x is the original state of the system, xH is the estimated state of the system, s is the original state of the transformed system,...
%% sH is the estimated state of the transformed system.
%Comparison Plot

solution = output.result.solution; %Solution result
time = solution.phase(1).time; % Output time series
state = solution.phase(1).state; % Output way points
control = solution.phase(1).control; % Controller trajectory
cost = solution.phase(1).integral; %Cost



%Plot for the Phase protrait trajectory of the transformed states
figure(1)
plot(state(:,1),state(:,3))
grid on
hold on
% print -dpng State1_Jpops.png
% save('Transformed_State_Trajectory_Jpops.mat')
% figure(2)
% plot(time,cost)


%Plot for the Transformed state trajectory
figure(2);
pp = plot(time,state,'-o');
xl = xlabel('$t$','Interpreter','LaTeX');
yl = ylabel('$(x(t),x^*(t))$','Interpreter','LaTeX');
ll = legend('$x(t)$','$x^*(t)$','Location','NorthWest');
set(xl,'Fontsize',18);
set(yl,'Fontsize',18);
set(ll,'Interpreter','LaTeX','Fontsize',20);
set(gca,'Fontsize',16,'FontName','Times');
set(pp,'LineWidth',1.25);
grid on
% print -dpng Time_vs_States_Jpops.png
% save('Time_vs_States_Jpops_robot_SE_new.mat')

