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
ADP1 = load('Comparison_ADP_SE.mat');
ADP = load('main_SE_MBRL_final.mat');
GPOPS = load('Time_vs_States_Jpops_SE.mat');


a1 = -7; A1 = 5; %Barrier Boundaries for x(1)-coordinate values will be remained within (a1,A1) 
a2 = -5; A2 = 7; %Barrier Boundaries for x(2)-coordinate values will be remained within (a2,A2) 

figure(1)
p=plot(ADP1.z(:,1),ADP1.z(:,2),GPOPS.state(:,1),GPOPS.state(:,2),'v',5,'Linewidth',1);
% p=plot(ADP.z(:,1),ADP.z(:,2),GPOPS.state(:,1),GPOPS.state(:,2),'v',5,'Linewidth',1);
% rectangle('position',[-7 -5 12 12])
% axis([-8 7 -7 8])
mrk1={'s','v','o','*','^','x','+','.','d'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel({'$s_{1}(t)$'},'interpreter','latex')
ylabel({'$s_{2}(t)$'},'interpreter','latex')
set(gca,'FontSize',25)
l=legend('OF-SMBRL','GPOPS II','interpreter','latex')
set(l,'Interpreter','latex','Location','northeast','NumColumns',1);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
grid on
% print -dpdf Time_Vs_TStates_RL_BF_SE_Optimal_Comparison.png
% print -dpdf Time_Vs_TStates_RL_BF_SE_Optimal_Comparison.pdf

% figure(2)
% p=plot(ADP.t,ADP.z(:,1:ADP.P.n),'Linewidth',1);
% mrk1={'s','v','o','*','^','x','+','.','d'};
% mrk=(mrk1(1,1:size(p,1)))';
% set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
% f_nummarkers(p,10,1);
% for i=1:size(p,1)
%     hasbehavior(p(i), 'legend', false);
% end
% xlabel('Time, t(s)','interpreter','latex')
% ylabel({'States, $x$(t)'},'interpreter','latex')
% set(gca,'FontSize',10)
% l=legend('$x_{1}$','$x_{2}$','interpreter','latex')
% set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPosition', [0 0 5 2]);
% grid on
% print -dpdf Time_Vs_States_MBRL_SE_Optimal.png
% print -dpdf Time_Vs_States_MBRL_SE_Optimal.pdf


% figure(3)
% p=plot(ADP.t,ADP.z(:,ADP.P.n+1:2*ADP.P.n,1),'Linewidth',1);
% mrk1={'s','v','o','*','^','x','+','.','d'};
% mrk=(mrk1(1,1:size(p,1)))';
% set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
% f_nummarkers(p,10,1);
% for i=1:size(p,1)
%     hasbehavior(p(i), 'legend', false);
% end
% xlabel('Time, t(s)','interpreter','latex')
% ylabel({'Estimates States, $\hat{x}$(t)'},'interpreter','latex')
% set(gca,'FontSize',10)
% l=legend('$\hat{x}_{1}$','$\hat{x}_{2}$','interpreter','latex')
% set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPosition', [0 0 5 2]);
% grid on 
% print -dpdf Time_Vs_TStates_MBRL_SE_Optimal.png
% print -dpdf Time_Vs_TStates_MBRL_SE_Optimal.pdf

figure(2)
p=plot(ADP.t,(ADP.z(:,1:ADP.P.n)-ADP.z(:,ADP.P.n+1:2*ADP.P.n,1)),'Linewidth',1);
axis([0 4 -0.5 2.5])
mrk1={'s','v','o','*','^','x','+','.','d'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,30,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('$t$','interpreter','latex')
ylabel({'$\tilde{x}(t)$'},'interpreter','latex')
set(gca,'FontSize',25)
l=legend('$\tilde{x}_{1}$','$\tilde{x}_{2}$','interpreter','latex')
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
grid on
% print -dpdf Time_Vs_States_Errors_MBRL_SE_Optimal.png
% print -dpdf Time_Vs_States_Errors_MBRL_SE_Optimal.pdf

figure(3)
p=plot(ADP.t,((ADP.z(:,6:11))),'Linewidth',1);
axis([0 10 -0.5 10])
mrk1={'s','v','o','*','^','x','+','.','d'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('$t$','interpreter','latex');
ylabel('$\hat{W}(t)$','interpreter','latex');
set(gca,'FontSize',25)
l = legend('$\hat{W}_{a_{1}}$','$\hat{W}_{a_{2}}$','$\hat{W}_{a_{3}}$','$\hat{W}_{c_{1}}$','$\hat{W}_{c_{2}}$','$\hat{W}_{c_{3}}$','interpreter','latex')
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches'); 
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
grid on
% print -dpdf Time_Vs_States_Weights_MBRL_SE_Optimal.png
% print -dpdf Time_Vs_States_Weights_MBRL_SE_Optimal.pdf


figure(4)
p=plot(ADP.z(:,1),ADP.z(:,2),'v',5,'Linewidth',1);
rectangle('position',[-7 -5 12 12])
axis([-8 7 -7 8])
% mrk1={'s','v','o','*','^','x','+','.','d'};
% mrk=(mrk1(1,1:size(p,1)))';
% set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
% f_nummarkers(p,10,1);
% for i=1:size(p,1)
%     hasbehavior(p(i), 'legend', false);
% end
xlabel({'$x_{1}(t)$'},'interpreter','latex')
ylabel({'$x_{2}(t)$'},'interpreter','latex')
set(gca,'FontSize',25)
% l=legend('Phase portrait, $x$','interpreter','latex')
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
grid on
% print -dpdf Phase_Portrait_MBRL_SE_Optimal.png
% print -dpdf Phase_Portrait_MBRL_SE_Optimal.pdf


% figure(5)
% plot(ADP.t,((ADP.z(:,6:11))),'Linewidth',1);
% 
% xlabel('Time t');
% ylabel('$\hat{W}$','interpreter','latex');
% legend('$\hat{W}_{a_{1}}$','$\hat{W}_{a_{2}}$','$\hat{W}_{a_{3}}$','$\hat{W}_{c_{1}}$','$\hat{W}_{c_{2}}$','$\hat{W}_{c_{3}}$','interpreter','latex')
% set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPosition', [0 0 5 2]);
% drawnow; 
% grid
% print -dpdf Time_Vs_TStates_MBRL_SE_Optimal.png
% print -dpdf Time_Vs_TStates_MBRL_SE_Optimal.pdf