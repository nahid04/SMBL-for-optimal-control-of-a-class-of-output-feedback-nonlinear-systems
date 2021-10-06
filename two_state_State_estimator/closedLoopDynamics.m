%% Paper title "Safety aware model-based reinforcement learning for optimal control of a class of output-feedback nonlinear systems"
%% arXiv:2110.00271
%% Coded by S M Nahid Mahmud, MS Grad Student, Oklahoma State University.
%% nahid.mahmud@okstate.edu
%% This code presents a two-state dynamical system for the parameter estimation
%% The notations are similar to the mentioned paper. 
%% Transformed system denotes the system we have after applying the barrier transformation to the original system.
%% In this document, x is the original state of the system, xH is the estimated state of the system, s is the original state of the transformed system,...
%% sH is the estimated state of the transformed system.

%% Main Function
function zDot = closedLoopDynamics(t,z,P)

% Optimizing Matrix/Integraing matrix >> This z matrix is the integration
% of the zDot matrix given at the end of this coding.  
x = z(1:P.n,1); % row 1-2 in z vector
xH = z(P.n+1:2*P.n,1); % row 3-4 in z vector
eta1 = z(2*P.n+1,1); % row 5 in z vector
WaH = z(2*P.n+1+1:2*P.n+1+P.W); % row 6-8 in z vector
WcH = z(2*P.n+1+P.W+1:2*P.n+1+2*P.W); % row 9-11 in z vector
Gamma = reshape(z(2*P.n+1+2*P.W+1:2*P.n+1+2*P.W+P.G^2),P.G,P.G); % row 12-20 in z vector
s = z(2*P.n+1+2*P.W+P.G^2+1: 2*P.n+1+2*P.W+P.G^2+P.n,1); % row 21-22 in z vector
sH = z( 2*P.n+1+2*P.W+P.G^2+P.n+1:2*P.n+1+2*P.W+P.G^2+2*P.n,1); % row 23-24 in z vector


%% BE Extrapolation - Terms in weight update laws that are evaluated along arbitrarily selected trajectories ki
theta = [1;-1;-0.5;0.5]; % Value of the unknown parameters 
%Creating a meshgrid to train to get the optimal actor weight and critic weight along the meshgrid
%points using BE extrapolation 
X = linspace(-4,4,10);
M = length(X)^2;
[XX,XY]=meshgrid(X,X);
XC=[reshape(XX,M,1) reshape(XY,M,1)]';


%initialization of the trained weights in the meshgrid 
clWc=zeros(size(WcH)); %initialization of the critic weights in the meshgrid
clWa=zeros(size(WaH)); %initialization of the actor weights in the meshgrid
CLmatrix=zeros(size(WcH,1),size(WcH,1)); %Matrixized
CLGamma=zeros(size(Gamma)); %initialization of the Gamma weights in the meshgrid

for i=1:M
    ki=XC(:,i); % Vectorized
    
a1 = -7; A1 = 5; %Barrier Boundaries for sii(1), sii(1)-coordinate values will be remained within (a1,A1) 
a2 = -5; A2 = 7; %Barrier Boundaries for sii(2), sii(2)-coordinate values will be remained within (a2,A2) 

%transforming initial x-coordinates to initial s-coordinates in the meshgrid. 
    si_1 =barrier(ki(1),a1,A1);
    si_2 =barrier(ki(2),a2,A2);
    si = [si_1;si_2];
    
%transforming initial s-coordinates to initial x-coordinates in the meshgrid. 
x1_si = a1*A1*((exp(si(1)))-1)/((a1*exp(si(1)))-A1);
x2_si = a2*A2*((exp(si(2)))-1)/((a2*exp(si(2)))-A2);


%Note: We do not need to do the barrier transformation in the meshgrid to train
%our controller as this method is an extrapolation method, and we are picking some random
%points (around the meshgrid) to train. Therefore, we just need to create a
%meshgrid to train, but here we are doing the barrier transformation just
%to mimic the environment, not sure whether it gives a better result or
%not. 

%Dynamics in the meshgrid 
F1_1_si =  (((a1^2*exp(si(1))) - (2*a1*A1) + (A1^2 * exp (-si(1)))) / (-a1*A1^2+A1*a1^2)) * x1_si;
F1_2_si =  (((a1^2*exp(si(1))) - (2*a1*A1) + (A1^2 * exp (-si(1)))) / (-a1*A1^2+A1*a1^2)) * x2_si;
F2_1_si=  (((a2^2*exp(si(2))) - (2*a2*A2) + (A2^2 * exp (-si(2)))) / ( A2*a2^2 -a2*A2^2)) * x1_si;
F2_2_si = (((a2^2*exp(si(2))) - (2*a2*A2) + (A2^2 * exp (-si(2)))) / ( A2*a2^2 -a2*A2^2))* x2_si;
F2_3_si = (((a2^2*exp(si(2))) - (2*a2*A2) + (A2^2 * exp (-si(2)))) / ( A2*a2^2 -a2*A2^2)) * (x2_si*(cos(2*x1_si)+2)^2);
G2_si = (((a2^2*exp(si(2))) - (2*a2*A2) + (A2^2 * exp (-si(2)))) / ( A2*a2^2 -a2*A2^2))* (cos(2*x1_si)+2);
Y_si = [F1_2_si,0,0,0;0,F2_1_si,F2_2_si,F2_3_si];
G_si = [0; G2_si];

%ADP-Controller
% Basis vector in the meshgrid: [si(1)^2, si(1)si(2),si(2)^2]
phi_p_si = [2*si(1) 0; si(2) si(1); 0 2*si(2)]; % Jacobian of the basis vector in the meshgrid
mu_si = -0.5.*(P.R\G_si')*phi_p_si'*WaH; % trained controller of the meshgrid
%Definition
omegai = phi_p_si*(Y_si*theta+G_si*mu_si);
Gsigmai = phi_p_si*G_si*(P.R\G_si')*phi_p_si';  
rhoi=(1+P.v*(omegai'*omegai));
 
%Cost function in the meshgrid 
 ri = si'*P.Q*si + mu_si'*P.R*mu_si;
% 
%     sig_pi =[2*xi(1) 0; xi(2) xi(1); 0 2*xi(2)]; 
%     mui=-0.5*(R\Gi')*sig_pi'*WaH;
%     omegai = sig_pi*(Fi+Gi*mui);
%     rhoi=(1+v*(omegai'*omegai));
%     ri = xi'*Q*xi + mui'*R*mui;
deltai = WcH'*omegai + ri;
% actor weight, critic weight, Gamma update law  
    clWc=clWc+Gamma*omegai*deltai/rhoi;
    clWa=clWa+Gsigmai'*WaH*omegai'*WcH/(4*rhoi);
    CLmatrix=CLmatrix+omegai*omegai'/(M*rhoi);
    CLGamma=CLGamma+Gamma*(omegai*omegai'/rhoi^2)*Gamma;
end
clWc=-P.kc*clWc/M;
clWa=P.kc*clWa/M;
CLGamma=CLGamma/M;
cbar =min(svd(CLmatrix));

%% Dynamics

%% Original dynamics without the parameters

Y1 = [x(2),0,0,0;0,x(1),x(2), x(2)*((cos(2*x(1))+2)^2)];
G1 = [0; (cos(2*x(1))+2)];

%% Estimated original dynamics without the parameters
YH1 = [xH(2),0,0,0;0,xH(1),xH(2), xH(2)*((cos(2*xH(1))+2)^2)];
GH1 = [0; (cos(2*xH(1))+2)];

%Known Parameters of the dynamics
theta = [1;-1;-.5;.5];


%Definition according to the paper
s1H = sH(1);
s1 = s(1);
s1Tilde = s1-s1H;
s2H = sH(2);
s2 = s(2);
s2Tilde = s2-s2H;
s = [s1;s2];
sH = [s1H;s2H];
P.s1Tilde_initial = P.s10-P.s1H0; 
eta = -eta1 -(P.k+P.alpha)*(s1Tilde-P.s1Tilde_initial);
nu1 = ((A1*a1^2-a1*A1^2)/(a1^2*exp(s1H)-2*a1*A1+A1^2*exp(-s1H)))*(P.alpha^2*(s1-s1H)-(P.k+P.alpha+P.beta1)*eta);
nu2 = ((a2^2*exp(s2H)-2*a2*A2+A2^2*exp(-s2H))/(A2*a2^2-a2*A2^2))*nu1;


%%transforming initial s-coordinates to initial x-coordinates. 

%%Transformed dynamics
F1_2s =  (((a1^2*exp(s(1))) - (2*a1*A1) + (A1^2 * exp (-s(1)))) / ( A1*a1^2 -a1*A1^2)) * x(2);
F2_1s =  (((a2^2*exp(s(2))) - (2*a2*A2) + (A2^2 * exp (-s(2)))) / ( A2*a2^2 -a2*A2^2)) * x(1);
F2_2s = (((a2^2*exp(s(2))) - (2*a2*A2) + (A2^2 * exp (-s(2)))) / ( A2*a2^2 -a2*A2^2)) * x(2);
F2_3s = (((a2^2*exp(s(2))) - (2*a2*A2) + (A2^2 * exp (-s(2)))) / ( A2*a2^2 -a2*A2^2)) * (x(2)*(cos(2*x(1))+2)^2);
G2s = (((a2^2*exp(s(2))) - (2*a2*A2) + (A2^2 * exp (-s(2)))) / ( A2*a2^2 -a2*A2^2)) * (cos(2*x(1))+2);
Y1s = [F1_2s,0,0,0;0,F2_1s,F2_2s,F2_3s];
G1s = [0; G2s];

%%Transformed estimated dynamics
F1_2Hs =  (((a1^2*exp(sH(1))) - (2*a1*A1) + (A1^2 * exp (-sH(1)))) / ( A1*a1^2 -a1*A1^2)) * xH(2);
F2_1Hs =  (((a2^2*exp(sH(2))) - (2*a2*A2) + (A2^2 * exp (-sH(2)))) / ( A2*a2^2 -a2*A2^2)) * xH(1);
F2_2Hs = (((a2^2*exp(sH(2))) - (2*a2*A2) + (A2^2 * exp (-sH(2)))) / ( A2*a2^2 -a2*A2^2)) * xH(2);
F2_3Hs = (((a2^2*exp(sH(2))) - (2*a2*A2) + (A2^2 * exp (-sH(2)))) / ( A2*a2^2 -a2*A2^2)) * (xH(2)*(cos(2*xH(1))+2)^2);
G2Hs = (((a2^2*exp(sH(2))) - (2*a2*A2) + (A2^2 * exp (-sH(2)))) / ( A2*a2^2 -a2*A2^2)) * (cos(2*xH(1))+2);
YH1s = [F1_2Hs,0,0,0;0,F2_1Hs,F2_2Hs,F2_3Hs];
GH1s = [0; G2Hs];


%%
%% Terms in weight update laws that are evaluated along the system trajectory x


%% Basis vector: [s1^2, s1s2,s2^2]

%% Basis vector: [s1H^2, s1Hs2H,s2H^2]
%% Jacobian of basis vector
phi_pH = [2*sH(1) 0; sH(2) sH(1); 0 2*sH(2)];

%Learned controller of the transformed system
muH = -0.5.*(P.R\GH1s')*phi_pH'*WaH;

%Time step checking
t

%Integrating function
zDot = [Y1*theta+G1*muH;
        YH1*theta+GH1*muH+[0;nu1];
       (P.beta1+P.k)*eta+P.k*P.alpha*s1Tilde;
        (P.ka1*(WcH-WaH))-(P.ka2*WaH)+clWa;
        clWc;
        reshape(P.beta*Gamma-CLGamma,P.G^2,1);
        Y1s*theta+G1s*muH;
        YH1s*theta+GH1s*muH+[0;nu2];
       ];
end