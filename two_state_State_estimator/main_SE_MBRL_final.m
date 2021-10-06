%% Paper title "Safety aware model-based reinforcement learning for optimal control of a class of output-feedback nonlinear systems"
%% arXiv:2110.00271
%% Coded by S M Nahid Mahmud, MS Grad Student, Oklahoma State University.
%% nahid.mahmud@okstate.edu
%% This code presents a two-state dynamical system for the parameter estimation
%% The notations are similar to the mentioned paper. 
%% Transformed system denotes the system we have after applying the barrier transformation to the original system.
%% In this document, x is the original state of the system, xH is the estimated state of the system, s is the original state of the transformed system,...
%% sH is the estimated state of the transformed system.
clear all
clc


tic
%% PreSim

a1 = -7; A1 = 5; %Barrier Boundaries for x1, x1-coordinate values will be remained within (a1,A1) %In paper, we have used a1 = -7; A1 = 5;
a2 = -5; A2 = 7; %Barrier Boundaries for x2: x1-coordinate values will be remained within (a2,A2) %In paper, we have used a2 = -5; A2 = 7;

P.n = 2; %number of states
P.m = 1; %number of dimension

P.x0 = [-6;6]; % Initial x-coordinate values, it has to be within (a,A). %In paper, we have used [-6.5;6.5]
P.xH0 = [-6;4]; % Initial x-coordinate values, it has to be within (a,A). %In paper, we have used [-5;5]

%transforming initial x-coordinates to initial s-coordinates. 
P.s10 = barrier(P.x0(1),a1,A1);
P.s20 = barrier(P.x0(2),a2,A2);
P.s0 = [P.s10;P.s20]; % Initial s-coordinate values

%transforming initial x-coordinates to initial s-coordinates. 
P.s1H0 = barrier(P.xH0(1),a1,A1);
P.s2H0 = barrier(P.xH0(2),a2,A2);
P.sH0 = [P.s1H0;P.s2H0]; % Initial s-coordinate values

P.s1Tilde_initial = P.s10-P.s1H0; % Difference between the initial s-coordinates and estimated s-coordinates. 

P.eta10 = 0; % Initial value of compensating signal.   
P.W = 3; % number of the Actor-Critic Weight vector elements.
P.G =3;  % number of the Gamma vector elements. 
P.Wa=[10;0.5;0.5];
P.Wc=[10;0.5;0.5];
% Control Penalty Matrix  
P.R=0.1; 
% State Penalty Matrix  
P.Q=[10 0; 0 10]; 

% Tuning Gains
%%%
P.k = 10;
P.alpha = 1;
P.beta1 = 5;
% % Feedback Tuning Gains
P.kc = 5;
P.ka1 = 100;
P.ka2 =0.1;
P.beta = 0.1;
P.v = 1;



%Initial values for x = 1-2, xhat = 3-4,  eta1= 5, waH
%= 6-8, WcH = 9-11, Gamma = 12-20, s = 21-22, shat = 23-24,
% WaH denotes Estimated Actor weight, WcH denotes Estimated Critic weight
P.z0 = [P.x0;P.xH0;P.eta10;P.Wa;P.Wc;1;0;0;0;1;0;0;0;1;P.s0;P.sH0];
% P.z0 = [P.x0;P.xH0;P.eta10;0.5*ones(2*P.W,1);1;0;0;0;1;0;0;0;1];

%% Integration
%ODE45 function
% options = odeset('OutputFcn',@odeplot,'OutputSel',P.n+1:P.n+P.l);
[t,z] = ode45(@(t,z) closedLoopDynamics(t,z,P),[0 20],P.z0); 

%%Plots 

% State trajectories of the x-coordinates
figure(1)
p = plot(t,z(:,1),t,z(:,2));
mrk1={'s','v','o','*','^','x','+','.','d'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('Time, t(s)','interpreter','latex');
ylabel({'$x$(t)'},'interpreter','latex');
set(gca,'FontSize',10);
l=legend('real, $x_{1}$','real, $x_{2}$','interpreter','latex');
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPosition', [0 0 5 2]);
drawnow; 
grid

% State trajectories of the estimated x-coordinates

figure(2)
p = plot(t,z(:,3),t,z(:,4));
mrk1={'s','v','o','*','^','x','+','.','d'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('Time, t(s)','interpreter','latex');
ylabel({'$x$(t)'},'interpreter','latex');
set(gca,'FontSize',10);
l=legend('estimated, $x_{1}$','estimated, $x_{2}$','interpreter','latex');
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPosition', [0 0 5 2]);
drawnow; 
grid

% Error between the original x-coordinates and the estimated
% x-coordinates trajectories

figure(3)
p=plot(t,(z(:,1)-z(:,3)),t,(z(:,2)-z(:,4)));
mrk1={'s','v','o','*','^','x','+','.','d'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('Time, t(s)','interpreter','latex');
ylabel({'$x$(t)'},'interpreter','latex');
set(gca,'FontSize',10);
l=legend('state errors, $x_{1}$','state errors, $x_{2}$','interpreter','latex');
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPosition', [0 0 5 2]);
drawnow; 
grid

% Trajectory of the Actor and Critic weights 

figure(4)
p = plot(t,z(:,6),t,z(:,7),t,z(:,8),t,z(:,9),t,z(:,10),t,z(:,11))
mrk1={'s','v','o','*','^','x','+','.','d'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('Time t');
ylabel('$\hat{W}$','interpreter','latex');
legend('$\hat{W}_{a_{1}}$','$\hat{W}_{a_{2}}$','$\hat{W}_{a_{3}}$','$\hat{W}_{c_{1}}$','$\hat{W}_{c_{2}}$','$\hat{W}_{c_{3}}$','interpreter','latex')
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPosition', [0 0 5 2]);
drawnow; 
grid
% 
% State trajectories of the s-coordinates
figure(5)
p = plot(t,z(:,2*P.n+1+2*P.W+P.G^2+1:2*P.n+1+2*P.W+P.G^2+P.n));
mrk1={'s','v','o','*','^','x','+','.','d'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('Time, t(s)','interpreter','latex');
ylabel({'$s$(t)'},'interpreter','latex');
set(gca,'FontSize',10);
l=legend('real, $s_{1}$','real, $s_{2}$','interpreter','latex');
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPosition', [0 0 5 2]);
drawnow; 
grid

% State trajectories of the estimated s-coordinates
figure(6)
p = plot(t,z(:,2*P.n+1+2*P.W+P.G^2+P.n+1:2*P.n+1+2*P.W+P.G^2+2*P.n));
mrk1={'s','v','o','*','^','x','+','.','d'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('Time, t(s)','interpreter','latex');
ylabel({'$s$(t)'},'interpreter','latex');
set(gca,'FontSize',10);
l=legend('Estimated, $s_{1}$','Estimated, $s_{2}$','interpreter','latex');
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPosition', [0 0 5 2]);
drawnow; 
grid


% State trajectories of the estimated s-coordinates
figure(7)
p = plot(t,z(:,2*P.n+1+2*P.W+P.G^2+P.n+1:2*P.n+1+2*P.W+P.G^2+2*P.n)-z(:,2*P.n+1+2*P.W+P.G^2+1:2*P.n+1+2*P.W+P.G^2+P.n));
mrk1={'s','v','o','*','^','x','+','.','d'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('Time, t(s)','interpreter','latex');
ylabel({'$s$(t)'},'interpreter','latex');
set(gca,'FontSize',10);
l=legend('Error, $\tilde{s}_{1}$','Error, $\tilde{s}_{2}$','interpreter','latex');
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPosition', [0 0 5 2]);
drawnow; 
grid




%%Saving data format to use later. 
% save 'main_SE_MBRL_final.mat';

toc

%% statistical comparison of the result 

% stepinfo((z(:,9)-z(:,6)),t,0,'SettlingTimeThreshold',0.05,'RiseTimeThreshold',[0.00 0.99])
% stepinfo((z(:,7)-z(:,10)),t,0,'SettlingTimeThreshold',0.05,'RiseTimeThreshold',[0.00 0.99])
% stepinfo((z(:,8)-z(:,11)),t,0,'SettlingTimeThreshold',0.05,'RiseTimeThreshold',[0.00 0.99])
% 
% stepinfo(z(:,3),t,0,'SettlingTimeThreshold',0.01,'RiseTimeThreshold',[0.00 0.99])
% stepinfo(z(:,4),t,0,'SettlingTimeThreshold',0.01,'RiseTimeThreshold',[0.00 0.99])
% 
% stepinfo((z(:,1)-z(:,3)),t,0,'SettlingTimeThreshold',0.01,'RiseTimeThreshold',[0.00 0.99])
% stepinfo((z(:,2)-z(:,4)),t,0,'SettlingTimeThreshold',0.01,'RiseTimeThreshold',[0.00 0.99])

% RMSE1= (sum((z(:,1)-z(:,3)))^(2)/(length(z(:,1))))^(0.5)
% RMSE2= (sum((z(:,2)-z(:,4)))^(2)/(length(z(:,1))))^(0.5)
%Errors

% Value of the weights to check

z(length(t),9:11)
