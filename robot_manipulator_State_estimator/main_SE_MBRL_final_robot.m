%% Paper title "Safety aware model-based reinforcement learning for optimal control of a class of output-feedback nonlinear systems"
%% arXiv:2110.00271
%% Coded by S M Nahid Mahmud, MS Grad Student, Oklahoma State University.
%% nahid.mahmud@okstate.edu
%% This code presents a four-state dynamical system for the parameter estimation
%% The notations are similar to the mentioned paper. 
%% Transformed system denotes the system we have after applying the barrier transformation to the original system.
%% In this document, x is the original state of the system, xH is the estimated state of the system, s is the original state of the transformed system,...
%% sH is the estimated state of the transformed system.

clear all
clc

tic
%% PreSim

a1 = -1; A1 = 1; %Barrier Boundaries for x1, x1-coordinate values will be remained within (a1,A1) 
a2 = -1; A2 = 1; %Barrier Boundaries for x2, x2-coordinate values will be remained within (a2,A2) 
a3 = -2; A3 = 2; %Barrier Boundaries for x3, x3-coordinate values will be remained within (a3,A3) 
a4 = -2; A4 = 2; %Barrier Boundaries for x4, x4-coordinate values will be remained within (a4,A4) 


P.n = 4; %number of states
P.d = 2; %fixed constant just to use later
P.m = 1; %number of dimension

P.x0 = [-.5;-.5;1;1]; % Initial x-coordinate values, it has to be within (a,A).
P.xH0 = [-.5;-.5;1.1;1.1];  % Initial x-coordinate values, it has to be within (a,A).

%transforming initial x-coordinates to initial s-coordinates. 
s10 = barrier(P.x0(1),a1,A1);
s20 = barrier(P.x0(2),a2,A2);
s30 = barrier(P.x0(3),a3,A3);
s40 = barrier(P.x0(4),a4,A4);
P.s0 = [s10;s20;s30;s40]; % Initial s-coordinate values

%transforming initial x-coordinates to initial s-coordinates. 
s1H0 = barrier(P.xH0(1),a1,A1);
s2H0 = barrier(P.xH0(2),a2,A2);
s3H0 = barrier(P.xH0(3),a3,A3);
s4H0 = barrier(P.xH0(4),a4,A4);
P.sH0 = [s1H0;s2H0;s3H0;s4H0]; % Initial s-coordinate values

%Initialize of the eta function. Check paper for more understanding
P.eta30 = 0;
P.eta40 = 0;

P.W = 10; % elements of the Actor-Critic Weight vector
P.G = 10;  % elements of the Gamma vector

P.Wa0 = [5; 15; 0; 0; 12; 2; 15; 8; 2; 2]; % Initial value of the Actor weight vector. 
P.Wc0 = [15; 15; 0; 0; 12; 2; 15; 8; 2; 2]; % Initial value of the Critic weight vector. 


%Initial values for x = 1-4, xH = 5-8, eta1 = 9-10, 
% WaH = 11-20, WcH = 21-30, Gamma = 31-130, s = 131-134, sH = 135-138.


% WaH denotes Estimated Actor weight, WcH denotes Estimated Critic weight.
P.z0 = [P.x0;P.xH0;P.eta30;P.eta40;P.Wa0;P.Wc0;1*reshape(eye(10),100,1);P.s0;P.sH0];
%% Integration
%ODE45 function
% options = odeset('OutputFcn',@odeplot,'OutputSel',P.n+1:P.n+P.l);
[t,z] = ode45(@(t,z) closedLoopDynamics(t,z,P),[0 70],P.z0); %ODE45 function

%% Plot

% State trajectories of the x-coordinates
figure(1)
plot(t,z(:,1:4))
xlabel('Time t');
ylabel('$x$','interpreter','latex');
legend('$x_{1}$','$x_{2}$','$x_{3}$','$x_{4}$','interpreter','latex')
drawnow; 
grid

% State trajectories of the estimated x-coordinates
figure(2)
plot(t,z(:,5:8))
xlabel('Time t');
ylabel('$x$','interpreter','latex');
legend('$\hat{x}_{1}$','$\hat{x}_{2}$','$\hat{x}_{3}$','$\hat{x}_{4}$','interpreter','latex')
drawnow; 
grid

% The trajectory of the error between the x-coordinates and the estimated x-coordinates
figure(3)
plot(t,(z(:,1:4)-z(:,5:8)))
xlabel('Time t');
ylabel('$x$','interpreter','latex');
legend('$\tilde{x}_{1}$','$\tilde{x}_{2}$','$\tilde{x}_{3}$','$\tilde{x}_{4}$','interpreter','latex')
drawnow; 
grid


figure(4)
plot(t,z(:,11:20))
% xlim([0 50])
% ylim([-100 100])
xlabel('Time t');
ylabel('$W_{a}$','interpreter','latex');
% legend('$Wa_{1}$','$\hat{x}_{2}$','$\hat{x}_{3}$','$\hat{x}_{4}$','interpreter','latex')
drawnow; 
grid

figure(5)
plot(t,z(:,21:30))

xlabel('Time t');
ylabel('$W_{c}$','interpreter','latex');
 legend('$W_{c_{1}}$','$W_{c_{2}}$','$W_{c_{3}}$','$W_{c_{4}}$','$W_{c_{5}}$','$W_{c_{6}}$','$W_{c_{7}}$','$W_{c_{8}}$','$W_{c_{9}}$','$W_{c_{10}}$','interpreter','latex')
drawnow; 
grid


figure(6)
plot(t,(z(:,11:20)-z(:,21:30)))

xlabel('Time t');
ylabel('$\tilde{W}$','interpreter','latex');
% legend('$\hat{x}_{1}$','$\hat{x}_{2}$','$\hat{x}_{3}$','$\hat{x}_{4}$','interpreter','latex')
drawnow; 
grid




% The trajectory of the error between the s-coordinates and the estimated x-coordinates
figure(7)
plot(t,(z(:,131:138)))
xlabel('Time t');
ylabel('$x$','interpreter','latex');
legend('$\tilde{x}_{1}$','$\tilde{x}_{2}$','$\tilde{x}_{3}$','$\tilde{x}_{4}$','interpreter','latex')
drawnow; 
grid

toc
% save 'new_test_robot.mat';

Cric = z(length(t),21:30)
% Act = z(length(t),11:20)
%Errors

function zDot = closedLoopDynamics(t,z,P)
R=10*[0.1,0;0,0.1]; % Control Penalty Matrix                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
Q=[10,0,0,0;0,10,0,0;0,0,10,0;0,0,0,10]; %Gain Matrix


% Tuning Gains>> Please check the ArXiv paper to understand the notation
kc = 1000;
ka1 = 100;
ka2 =0.5;
beta = 0.001;
v = 500;

% State estimation gains
k =50;
alpha = 1 ;
beta1 = 10;



a1 = -1; A1 = 1; %Barrier Boundaries for x1, x1-coordinate values will be remained within (a1,A1) 
a2 = -1; A2 = 1; %Barrier Boundaries for x2, x2-coordinate values will be remained within (a2,A2) 
a3 = -2; A3 = 2; %Barrier Boundaries for x3, x3-coordinate values will be remained within (a3,A3) 
a4 = -2; A4 = 2; %Barrier Boundaries for x4, x4-coordinate values will be remained within (a4,A4) 


% Optimizing Matrix/Integraing matrix >> This z matrix is the integration
% of the zDot matrix given at the end of this coding. 
x = z(1:P.n,1);% row 1-4 in z vector
xH = z(P.n+1:2*P.n,1);% row 5-8 in z vector 
eta1 = z(2*P.n+1:2*P.n+P.d,1);% row 9-10 in z vector 
WaH = z(2*P.n+P.d+1:2*P.n+P.d+P.W); % row 11-20 in z vector
WcH = z(2*P.n+P.d+P.W+1:2*P.n+P.d+2*P.W); % row 21-30 in z vector
Gamma = reshape(z(2*P.n+P.d+2*P.W+1:2*P.n+P.d+2*P.W+P.G^2),P.G,P.G);  % row 31-130 in z vector 
s = z(2*P.n+P.d+2*P.W+P.G^2+1: 2*P.n+P.d+2*P.W+P.G^2+P.n,1); % row 131-134 in z vector
sH = z( 2*P.n+P.d+2*P.W+P.G^2+P.n+1:2*P.n+P.d+2*P.W+P.G^2+2*P.n,1); % row 135-138 in z vector

%% BE Extrapolation - Terms in weight update laws that are evaluated along arbitrarily selected trajectories xi

X = linspace(-.9,.9,5);
M = length(X)^4;
[XX,XY,XZ,XA]=ndgrid(X,X,X,X);
XC=[reshape(XX,M,1) reshape(XX,M,1) reshape(XY,M,1) reshape(XA,M,1)]';

%initialization of the trained weights in the meshgrid 
clWc=zeros(size(WcH)); %initialization of the critic weights in the meshgrid
clWa=zeros(size(WaH)); %initialization of the actor weights in the meshgrid
CLmatrix=zeros(size(WcH,1),size(WcH,1)); %Matrixized
CLGamma=zeros(size(Gamma));  %initialization of the Gamma weights in the meshgrid
for i=1:M
    ki=XC(:,i);
   
a1 = -1; A1 = 1;
a2 = -1; A2 = 1;
a3 = -2; A3 = 2;
a4 = -2; A4 = 2;% Barrier Boundaries
%% Barrier Boundaries % Barrier Boundaries

    %transforming initial x-coordinates to initial s-coordinates. 
    si_1 =barrier(ki(1),a1,A1);
    si_2 =barrier(ki(2),a2,A2);
    si_3 =barrier(ki(3),a3,A3);
    si_4 =barrier(ki(4),a4,A4);
    xi = [si_1;si_2;si_3;si_4];
%% Defining dynamics in the meshgrid, check ArxiV paper to understand the dynamics    
    s2i = sin(xi(2));
    c2i = cos(xi(2));
    
    p1i = 3.473;
    p2i = 0.196;
    p3i = 0.242;
    
    Mi = [p1i+2*p3i*c2i,p2i+p3i*c2i;p2i+p3i*c2i,p2i];
    inv_Mi = inv(Mi);
     Di = [xi(3) 0 0 0; 0 xi(4) 0 0; 0 0 tanh(xi(3)) 0; 0 0 0 tanh(xi(4))];
    Vm_i = [-p3i*s2i*xi(4), -p3i*s2i*(xi(3)+xi(4));p3i*s2i*xi(3),0];
    
    Gi = [0 0; 0 0;(inv_Mi)'];
    F1i = [xi(3);xi(4);-inv_Mi*Vm_i*[xi(3);xi(4)]];
    F2i = [0 0 0 0;0 0 0 0; -[inv_Mi,inv_Mi]*Di];
    thetai = [5.3;1.1;8.45;2.35];
    
    sig_pi = [xi(3),0,xi(1),0;0,xi(4),0,xi(2);...
    0,xi(3),xi(2),0;xi(4),0,0,xi(1);...
    xi(2),xi(1),0,0;0,0,xi(4),xi(3);...
    2*xi(1),0,0,0;0,2*xi(2),0,0;...
    0,0,2*xi(3),0;0,0,0,2*xi(4)];


%Note: We do not need to do the barrier transformation in the meshgrid to train
%our controller as this method is an extrapolation method, and we are picking some random
%points (around the meshgrid) to train. Therefore, we just need to create a
%meshgrid to train, but here we are doing the barrier transformation just
%to mimic the environment, not sure whether it gives a better result or
%not. 

%Definition
    mui=-0.5*(R\Gi')*sig_pi'*WaH;
    omegai = sig_pi*(F1i+F2i*thetai+Gi*mui);
    rhoi=(1+v*(omegai'*omegai));
    ri = xi'*Q*xi + mui'*R*mui;
    deltai = WcH'*omegai + ri;
% actor weight, critic weight, Gamma update law   
    clWc=clWc+Gamma*omegai*deltai/rhoi;
    Gsigmai = sig_pi*Gi*(R\Gi')*sig_pi';
    clWa=clWa+Gsigmai'*WaH*omegai'*WcH/(4*rhoi);
    CLmatrix=CLmatrix+omegai*omegai'/(M*rhoi);
    CLGamma=CLGamma+Gamma*(omegai*omegai'/rhoi^2)*Gamma;
end
clWc=-kc*clWc/M;
clWa=kc*clWa/M;
CLGamma=CLGamma/M;
cbar=min(svd(CLmatrix));

%% Dynamics

%%
% known parameters,
p1 = 3.473;
p2 = 0.196;
p3 = 0.242;
%% Defining dynamics, check ArxiV paper to understand the dynamics
s2 = sin(x(2));
c2 = cos(x(2));
s2H = sin(xH(2));
c2H = cos(xH(2));
 
M = [p1+2*p3*c2,p2+p3*c2;p2+p3*c2,p2];
MH = [p1+2*p3*c2H,p2+p3*c2H;p2+p3*c2H,p2];
inv_M = inv(M);
inv_MH = inv(MH);
D = [x(3) 0 0 0; 0 x(4) 0 0; 0 0 tanh(x(3)) 0; 0 0 0 tanh(x(4))];
DH = [xH(3) 0 0 0; 0 xH(4) 0 0; 0 0 tanh(xH(3)) 0; 0 0 0 tanh(xH(4))];

Vm = [-p3*s2*x(4), -p3*s2*(x(3)+x(4));p3*s2*x(3),0];
VmH = [-p3*s2H*xH(4), -p3*s2H*(xH(3)+xH(4));p3*s2H*xH(3),0];
    
G = [0 0; 0 0;(inv_M)'];
GH = [0 0; 0 0;(inv_MH)'];

F1 = [x(3);x(4);-inv_M*Vm*[x(3);x(4)]];
F1H = [xH(3);xH(4);-inv_MH*VmH*[xH(3);xH(4)]];

F2 = [0 0 0 0;0 0 0 0; -[inv_M,inv_M]*D];
F2H = [0 0 0 0;0 0 0 0; -[inv_MH,inv_MH]*DH];

theta = [5.3;1.1;8.45;2.35]; %known parameter

%transforming initial x-coordinates to initial s-coordinates. 
s1H = barrier(xH(1),a1,A1);
s1 = barrier(x(1),a1,A1);
s1Tilde = s1-s1H;
s2H = barrier(xH(2),a2,A2);
s2 = barrier(x(2),a2,A2);
s2Tilde = s2-s2H;
s3H = barrier(xH(3),a3,A3);
s3 = barrier(x(3),a3,A3);
s3Tilde = s3-s3H;
s4H = barrier(xH(4),a4,A4);
s4 = barrier(x(4),a4,A4);
s4Tilde = s4-s4H;
qTilde = [s1Tilde;s2Tilde];
s = [s1;s2;s3;s4];
sH = [s1H;s2H;s3H;s4H];
%%

%Transformation matrix
Ts = [(((a1^2*exp(s(1))) - (2*a1*A1) + (A1^2 * exp (-s(1)))) / ( A1*a1^2 -a1*A1^2));
    (((a2^2*exp(s(2))) - (2*a2*A2) + (A2^2 * exp (-s(2)))) / ( A2*a2^2 -a2*A2^2));
    (((a3^2*exp(s(3))) - (2*a3*A3) + (A3^2 * exp (-s(3)))) / ( A3*a3^2 -a3*A3^2));
    (((a4^2*exp(s(4))) - (2*a4*A4) + (A4^2 * exp (-s(4)))) / ( A4*a4^2 -a4*A4^2))];

TsH = [(((a1^2*exp(sH(1))) - (2*a1*A1) + (A1^2 * exp (-sH(1)))) / ( A1*a1^2 -a1*A1^2));
        (((a2^2*exp(sH(2))) - (2*a2*A2) + (A2^2 * exp (-sH(2)))) / ( A2*a2^2 -a2*A2^2));
        (((a3^2*exp(sH(3))) - (2*a3*A3) + (A3^2 * exp (-sH(3)))) / ( A3*a3^2 -a3*A3^2));
        (((a4^2*exp(sH(4))) - (2*a4*A4) + (A4^2 * exp (-sH(4)))) / ( A4*a4^2 -a4*A4^2))];

%% Transformed Dynamics      
F1s = Ts.*[x(3);x(4);-inv_M*Vm*[x(3);x(4)]];
F1Hs = TsH.*[xH(3);xH(4);-inv_MH*VmH*[xH(3);xH(4)]];

F2s = Ts.*[0 0 0 0;0 0 0 0; -[inv_M,inv_M]*D];
F2Hs = TsH.*[0 0 0 0;0 0 0 0; -[inv_MH,inv_MH]*DH];

Gs = Ts.*[0 0; 0 0; (inv_M)'];
GHs = TsH.*[0 0;0 0; (inv_MH)'];



%%

%%
%% Terms in weight update laws that are evaluated along the system trajectory x

% Jacobian of estimated basis vector
phi_pH = [sH(3),0,sH(1),0;0,sH(4),0,sH(2);...
    0,sH(3),sH(2),0;sH(4),0,0,sH(1);...
    sH(2),sH(1),0,0;0,0,sH(4),sH(3);...
    2*sH(1),0,0,0;0,2*sH(2),0,0;...
    0,0,2*sH(3),0;0,0,0,2*sH(4)];

% controller of the transformed system
muH = -0.5.*(R\GHs')*phi_pH'*WaH;

%Check definitions from the Arxiv paper
eta = -eta1 -(k+alpha)*qTilde;
nu3 = ((A1*a1^2-a1*A1^2)/(a1^2*exp(s1H)-2*a1*A1+A1^2*exp(-s1H)))*(alpha^2*(s1-s1H)-(k+alpha+beta1)*eta(1));  
nu4 =((A2*a2^2-a2*A2^2)/(a2^2*exp(s2H)-2*a2*A2+A2^2*exp(-s2H)))*(alpha^2*(s2-s2H)-(k+alpha+beta1)*eta(2));  
nu3s = ((a3^2*exp(s3H)-2*a3*A3+A3^2*exp(-s3H))/(A3*a3^2-a3*A3^2))*nu3;
nu4s = ((a4^2*exp(s4H)-2*a4*A4+A4^2*exp(-s4H))/(A4*a4^2-a4*A4^2))*nu4;

%Time step checking
t

%ODE function
zDot = [F1+F2*theta+G*muH;
        F1H+F2H*theta+GH*muH+[0;0;nu3;nu4];
        (beta1+k)*eta+(k*alpha)*qTilde;
        ka1*(WcH-WaH)-ka2*WaH+clWa;
        clWc;
        reshape(beta*Gamma-CLGamma,P.G^2,1);
        F1s+F2s*theta+Gs*muH;
        F1Hs+F2Hs*theta+GHs*muH+[0;0;nu3s;nu4s];];
end
