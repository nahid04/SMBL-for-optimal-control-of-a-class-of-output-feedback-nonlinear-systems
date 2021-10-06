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

%transforming initial x-coordinates to initial s-coordinates. 
P.s10 = barrier(P.x0(1),a1,A1);
P.s20 = barrier(P.x0(2),a2,A2);
P.s0 = [P.s10;P.s20]; % Initial s-coordinate values
P.cost0 = 0; % Initial cost

%Initial values for x = 1-2, Cost = 3
P.z0 = [P.s0;P.cost0];

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
% save('Comparison_ADP_SE.mat')

% State trajectories of the estimated x-coordinates

figure(2)
p = plot(t,z(:,3))
xlabel('Time, t(s)','interpreter','latex');
ylabel({'$r$(t)'},'interpreter','latex');
set(gca,'FontSize',10);
l=legend('cost, $r$','interpreter','latex');
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPosition', [0 0 5 2]);
drawnow; 
grid



toc



%% Main Function
function zDot = closedLoopDynamics(t,z,P)
a1 = -7; A1 = 5; %Barrier Boundaries for x1, x1-coordinate values will be remained within (a1,A1) %In paper, we have used a1 = -7; A1 = 5;
a2 = -5; A2 = 7; %Barrier Boundaries for x2: x1-coordinate values will be remained within (a2,A2) %In paper, we have used a2 = -5; A2 = 7;
% Control Penalty Matrix  
R=0.1; 
% State Penalty Matrix  
Q=[10 0; 0 10]; 
% Optimizing Vector
s = z(1:P.n,1); % row 1-2 in z vector
cost = z(P.n+1,1) % row 3 in z vector



%% BE Extrapolation - Terms in weight update laws that are evaluated along arbitrarily selected trajectories ki
theta = [1;-1;-0.5;0.5]; % Value of the unknown parameters 


%% Dynamics
s1 = s(1);
s2 = s(2);
s = [s1;s2];
%% Original dynamics without the parameters
x1 = invbarrier(s1,a1,A1);
x2 = invbarrier(s2,a2,A2);
x = [x1;x2];
Y1 = [x(2),0,0,0;0,x(1),x(2), x(2)*((cos(2*x(1))+2)^2)];
G1 = [0; (cos(2*x(1))+2)];

%Definition according to the paper
s1 = s(1);
s2 = s(2);
s = [s1;s2];


%%transforming initial s-coordinates to initial x-coordinates. 

%%Transformed dynamics
F1_2s =  (((a1^2*exp(s(1))) - (2*a1*A1) + (A1^2 * exp (-s(1)))) / ( A1*a1^2 -a1*A1^2)) * x(2);
F2_1s =  (((a2^2*exp(s(2))) - (2*a2*A2) + (A2^2 * exp (-s(2)))) / ( A2*a2^2 -a2*A2^2)) * x(1);
F2_2s = (((a2^2*exp(s(2))) - (2*a2*A2) + (A2^2 * exp (-s(2)))) / ( A2*a2^2 -a2*A2^2)) * x(2);
F2_3s = (((a2^2*exp(s(2))) - (2*a2*A2) + (A2^2 * exp (-s(2)))) / ( A2*a2^2 -a2*A2^2)) * (x(2)*(cos(2*x(1))+2)^2);
G2s = (((a2^2*exp(s(2))) - (2*a2*A2) + (A2^2 * exp (-s(2)))) / ( A2*a2^2 -a2*A2^2)) * (cos(2*x(1))+2);
Y1s = [F1_2s,0,0,0;0,F2_1s,F2_2s,F2_3s];
G1s = [0; G2s];



%%
%% Terms in weight update laws that are evaluated along the system trajectory x
%% Basis vector: [s1^2, s1s2,s2^2]
%% Jacobian of basis vector
phi_p = [2*s(1) 0; s(2) s(1); 0 2*s(2)];

% Fixed Weights
WaH = [ 8.6966    2.3385    1.8043]'; %Change according to the value of the Actor-Critic weights learnt from the simulation run.

%Learned controller of the original system
mu = -0.5.*(R\G1s')*phi_p'*WaH;

%Time step checking
t

zDot = [Y1s*theta+G1s*mu;
        s'*Q*s+mu*R*mu;
       ];
end