%% Paper title "Safety aware model-based reinforcement learning for optimal control of a class of output-feedback nonlinear systems"
%% arXiv:2110.00271
%% Coded by S M Nahid Mahmud, MS Grad Student, Oklahoma State University.
%% nahid.mahmud@okstate.edu
%% This code presents a four-state dynamical system for the parameter estimation
%% The notations are similar to the mentioned paper. 
%% Transformed system denotes the system we have after applying the barrier transformation to the original system.
%% In this document, x is the original state of the system, xH is the estimated state of the system, s is the original state of the transformed system,...
%% sH is the estimated state of the transformed system.

%% This code represents the cost of the developed framework using fixed weights in the controller. 
clear all
clc

tic
%% PreSim
a1 = -1; A1 = 1; %Barrier Boundaries for x1, x1-coordinate values will be remained within (a1,A1) 
a2 = -1; A2 = 1; %Barrier Boundaries for x2, x2-coordinate values will be remained within (a2,A2) 
a3 = -2; A3 = 2; %Barrier Boundaries for x3, x3-coordinate values will be remained within (a3,A3) 
a4 = -2; A4 = 2; %Barrier Boundaries for x4, x4-coordinate values will be remained within (a4,A4) 

P.n = 4; %number of states
P.d = 2; %number
P.m = 1; %number of dimension

P.x0 = [-.5;-.5;1;1]; % Initial x-coordinate values, it has to be within (a,A).

%transforming initial x-coordinates to initial s-coordinates. 
s10 = barrier(P.x0(1),a1,A1);
s20 = barrier(P.x0(2),a2,A2);
s30 = barrier(P.x0(3),a3,A3);
s40 = barrier(P.x0(4),a4,A4);
P.s0 = [s10;s20;s30;s40]; % Initial s-coordinate values

%transforming initial x-coordinates to initial s-coordinates. 


%Initialize of the eta function. Check paper for more understanding
P.eta30 = 0;
P.eta40 = 0;

%Initial values for s = 1-4, cost = 5.
P.z0 = [P.s0;0];
%ODE45 function
% options = odeset('OutputFcn',@odeplot,'OutputSel',P.n+1:P.n+P.l);
[t,z] = ode45(@(t,z) closedLoopDynamics(t,z,P),[0 30],P.z0); %ODE45 function

% save Comparison_ADP_SE_robot.mat
function zDot = closedLoopDynamics(t,z,P)
R=10*[0.1,0;0,0.1]; % Control Penalty Matrix                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
Q=[10,0,0,0;0,10,0,0;0,0,10,0;0,0,0,10]; %Gain Matrix

a1 = -1; A1 = 1; %Barrier Boundaries for x1, x1-coordinate values will be remained within (a1,A1) 
a2 = -1; A2 = 1; %Barrier Boundaries for x2, x2-coordinate values will be remained within (a2,A2) 
a3 = -2; A3 = 2; %Barrier Boundaries for x3, x3-coordinate values will be remained within (a3,A3) 
a4 = -2; A4 = 2; %Barrier Boundaries for x4, x4-coordinate values will be remained within (a4,A4) 

% Optimizing Matrix
s = z(1:P.n,1); % row 1-4 in z vector
reward = z(P.n+1) % row 5 in z vector

%transforming s-coordinates to x-coordinates. 
x1 = invbarrier(s(1),a1,A1);
x2 = invbarrier(s(2),a2,A2);
x3 = invbarrier(s(3),a3,A3);
x4 = invbarrier(s(4),a4,A4);
x = [x1;x2;x3;x4];

%Dynamics
s2 = sin(x(2));
c2 = cos(x(2));
p1 = 3.473;
p2 = 0.196;
p3 = 0.242;

M = [p1+2*p3*c2,p2+p3*c2;p2+p3*c2,p2];
inv_M = inv(M);
D = [x(3) 0 0 0; 0 x(4) 0 0; 0 0 tanh(x(3)) 0; 0 0 0 tanh(x(4))];
Vm = [-p3*s2*x(4), -p3*s2*(x(3)+x(4));p3*s2*x(3),0];
theta = [5.3;1.1;8.45;2.35];

%transforming s-coordinates to x-coordinates. 
s1 = barrier(x(1),a1,A1);
s2 = barrier(x(2),a2,A2);
s3 = barrier(x(3),a3,A3);
s4 = barrier(x(4),a4,A4);
s = [s1;s2;s3;s4];

%%

%Transformation matrix

Ts = [(((a1^2*exp(s(1))) - (2*a1*A1) + (A1^2 * exp (-s(1)))) / ( A1*a1^2 -a1*A1^2));
    (((a2^2*exp(s(2))) - (2*a2*A2) + (A2^2 * exp (-s(2)))) / ( A2*a2^2 -a2*A2^2));
    (((a3^2*exp(s(3))) - (2*a3*A3) + (A3^2 * exp (-s(3)))) / ( A3*a3^2 -a3*A3^2));
    (((a4^2*exp(s(4))) - (2*a4*A4) + (A4^2 * exp (-s(4)))) / ( A4*a4^2 -a4*A4^2))];

%Transformed Matrix
F1s = Ts.*[x(3);x(4);-inv_M*Vm*[x(3);x(4)]];
F2s = Ts.*[0 0 0 0;0 0 0 0; -[inv_M,inv_M]*D];
Gs = Ts.*[0 0; 0 0; (inv_M)'];

%% Terms in weight update laws that are evaluated along the system trajectory x

% Jacobian of basis vector
phi_p = [s(3),0,s(1),0;0,s(4),0,s(2);...
    0,s(3),s(2),0;s(4),0,0,s(1);...
    s(2),s(1),0,0;0,0,s(4),s(3);...
    2*s(1),0,0,0;0,2*s(2),0,0;...
    0,0,2*s(3),0;0,0,0,2*s(4)];

%Fixed Weight
F_WaH = [  
    14.5148    8.3241   -0.0937   -6.6335   12.1656    0.7793   14.9801    8.3511    4.8148    0.4593
]';

%Cost
mu = -0.5.*(R\Gs')*phi_p'*F_WaH;

%Time step checking
t

%ODE function
zDot = [F1s+F2s*theta+Gs*mu;
       s'*Q*s + mu'*R*mu;
];
end
