%% Paper title "Safety aware model-based reinforcement learning for optimal control of a class of output-feedback nonlinear systems"
%% arXiv:2110.00271
%% Coded by S M Nahid Mahmud, MS Grad Student, Oklahoma State University.
%% nahid.mahmud@okstate.edu
%% This code presents a four-state dynamical system for the parameter estimation
%% The notations are similar to the mentioned paper. 
%% Transformed system denotes the system we have after applying the barrier transformation to the original system.
%% In this document, x is the original state of the system, xH is the estimated state of the system, s is the original state of the transformed system,...
%% sH is the estimated state of the transformed system.

% Plots need to replicate the Arxiv paper! 



clear all
clc

ADP = load('new_test_robot.mat');
GPOPS = load('Time_vs_States_Jpops_robot_SE_new.mat');
ADP1 = load('Comparison_ADP_SE_robot.mat');

% Barrier Boundaries;
a1 = -1; A1 = 1;
a2 = -1; A2 = 1;
a3 = -2; A3 = 2;
a4 = -2; A4 = 2;

% State trajectories of the s-coordinates using OF-SMBRL using the fixed learned weight and GPOPS II
figure(1)
p = plot(ADP1.t,ADP1.z(:,1:2),GPOPS.time,GPOPS.state(:,1:2),'Linewidth',1);
mrk1={'s','v','o','*','^','x','+','.','d','h'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,20,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
 end

ylabel({'$s(t)$'},'interpreter','latex')
xlabel('$t$','interpreter','latex')
xlim([0 15])
ylim([-1.1 0.1])
set(gca,'FontSize',25)
l=legend('OF-SMBRL, $s_{1}$','OF-SMBRL, $s_{2}$',...
    'GPOPS II, $s_{1}$','GPOPS II, $s_{2}$','interpreter','latex')
set(l,'Interpreter','latex','Location','southeast','NumColumns',1);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
grid on
% print -dpdf Time_Vs_TStates_RL_BF_SE_Optimal_Comparison_robot_1.png
% print -dpdf Time_Vs_TStates_RL_BF_SE_Optimal_Comparison_robot_1.pdf

figure(2)
p = plot(ADP1.t,ADP1.z(:,3:4),GPOPS.time,GPOPS.state(:,3:4),'Linewidth',1);
mrk1={'s','v','o','*','^','x','+','.','d','h'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,20,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
 end
ylabel({'$s(t)$'},'interpreter','latex')
xlabel('$t$','interpreter','latex')
xlim([0 8])
ylim([-0.1 1.1])
set(gca,'FontSize',25)
l=legend('OF-SMBRL, $s_{3}$','OF-SMBRL, $s_{4}$',...
    'GPOPS II, $s_{3}$','GPOPS II, $s_{4}$','interpreter','latex')
set(l,'Interpreter','latex','Location','northeast','NumColumns',1);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
grid on
% print -dpdf Time_Vs_TStates_RL_BF_SE_Optimal_Comparison_robot_2.png
% print -dpdf Time_Vs_TStates_RL_BF_SE_Optimal_Comparison_robot_2.pdf






% 
% figure(2)
% p = plot(ADP1.t,ADP1.z(:,3),'Linewidth',1);
% hold on
% p = plot(ADP1.t,ADP1.z(:,4),'Linewidth',1);
% 
% hold on
% p = plot(GPOPS.time,GPOPS.state(:,3),'Linewidth',1);
% hold on
% p = plot(GPOPS.time,GPOPS.state(:,4),'Linewidth',1);
% 
% % p=plot(ADP1.t,barrier(ADP1.z(:,2),a2,A2),...
% %     barrier(ADP1.z(:,3),a3,A3),barrier(ADP1.z(:,4),a4,A4),...
% %     GPOPS.state(:,1),GPOPS.state(:,2),...
% %     GPOPS.state(:,3),GPOPS.state(:,4)),'v',5,'Linewidth',1);
% 
% % p=plot(ADP.z(:,1),ADP.z(:,2),GPOPS.state(:,1),GPOPS.state(:,2),'v',5,'Linewidth',1);
% % rectangle('position',[-7 -5 12 12])
% % axis([-8 7 -7 8])
% % mrk1={'s','v','o','*','^','x','+','.','d'};
% % mrk=(mrk1(1,1:size(p,1)))';
% % set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
% % f_nummarkers(p,10,1);
% % for i=1:size(p,1)
% %     hasbehavior(p(i), 'legend', false);
% % end
% xlabel({'State, s(t)'},'interpreter','latex')
% ylabel({'Time, t(s)'},'interpreter','latex')
% xlim([0 15])
% ylim([-3 3])
% set(gca,'FontSize',10)
% l=legend('BT MBRL, $s_{1}$','BT MBRL, $s_{2}$','BT MBRL, $s_{3}$','BT MBRL, $s_{4}$',...
%     'GPOPS II, $s_{1}$','GPOPS II, $s_{2}$','GPOPS II, $s_{3}$','GPOPS II, $s_{4}$','interpreter','latex')
% set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPosition', [0 0 5 2]);
% grid on
% print -dpng Time_Vs_TStates_RL_BF_SE_Optimal_Comparison_robot_2.png
% print -dpng Time_Vs_TStates_RL_BF_SE_Optimal_Comparison_robot_2.pdf


% State trajectories of the x-coordinates
figure(3)
p=plot(ADP.t,ADP.z(:,1:4),'Linewidth',1);
mrk1={'s','v','o','*','^','x','+','.','d','h'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,20,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('$t$','interpreter','latex')
ylabel({'$x(t)$'},'interpreter','latex')
xlim([0 10])
ylim([-2.5 2.5])
set(gca,'FontSize',25)
hold on
p1=plot(ADP.t,-1*ones(size(ADP.t)),'--','LineWidth',1);
set(p1,'Color',get(p(1),'Color'),'LineWidth',1)
hasbehavior(p1, 'legend', false);
p1=plot(ADP.t,1*ones(size(ADP.t)),'--','LineWidth',1);
set(p1,'Color',get(p(2),'Color'),'LineWidth',1)
hasbehavior(p1, 'legend', false);
p1=plot(ADP.t,2*ones(size(ADP.t)),'--','LineWidth',1);
set(p1,'Color',get(p(3),'Color'),'LineWidth',1)
hasbehavior(p1, 'legend', false);
p1=plot(ADP.t,-2*ones(size(ADP.t)),'--','LineWidth',1);
set(p1,'Color',get(p(4),'Color'),'LineWidth',1)
hasbehavior(p1, 'legend', false);
l=legend('$x_{1}$','$x_{2}$','$x_{3}$','$x_{4}$','interpreter','latex')
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
grid on
% print -dpdf Time_Vs_States_MBRL_SE_Optimal_robot_1.png
% print -dpdf Time_Vs_States_MBRL_SE_Optimal_robot_1.pdf



% figure(4)
% p=plot(ADP.t,ADP.z(:,3:ADP.P.n),'Linewidth',2);
% mrk1={'s','v','o','*','^','x','+','.','d','h'};
% mrk=(mrk1(1,1:size(p,1)))';
% set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',8);
% f_nummarkers(p,10,1);
% for i=1:size(p,1)
%     hasbehavior(p(i), 'legend', false);
% end
% xlabel('Time, t(s)','interpreter','latex')
% ylabel({'States, $x$(t)'},'interpreter','latex')
% xlim([0 6])
% ylim([-3 3])
% set(gca,'FontSize',10)
% hold on
% p1=plot(ADP.t,2*ones(size(ADP.t)),'--','LineWidth',2);
% set(p1,'Color',get(p(1),'Color'),'LineWidth',2)
% hasbehavior(p1, 'legend', false);
% p1=plot(ADP.t,-2*ones(size(ADP.t)),'--','LineWidth',2);
% set(p1,'Color',get(p(2),'Color'),'LineWidth',2)
% hasbehavior(p1, 'legend', false);
% l=legend('$x_{3}$','$x_{4}$','interpreter','latex')
% set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [8 4]);
% set(gcf, 'PaperPosition', [0 0 5 2]);
% grid on
% print -dpdf Time_Vs_States_MBRL_SE_Optimal_robot_2.png
% print -dpdf Time_Vs_States_MBRL_SE_Optimal_robot_2.pdf





% figure(4)
% p=plot(ADP.t,ADP.z(:,ADP.P.n+1:2*ADP.P.n,1),'Linewidth',2);
% mrk1={'s','v','o','*','^','x','+','.','d'};
% mrk=(mrk1(1,1:size(p,1)))';
% set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
% f_nummarkers(p,10,1);
% for i=1:size(p,1)
%     hasbehavior(p(i), 'legend', false);
% end
% xlabel('Time, t(s)','interpreter','latex')
% ylabel({'Estimates States, $\hat{x}$(t)'},'interpreter','latex')
% ylim([-3 3])
% set(gca,'FontSize',10)
% hold on
% p1=plot(ADP.t,1*ones(size(ADP.t)),'--','LineWidth',2);
% set(p1,'Color',get(p(1),'Color'),'LineWidth',2)
% hasbehavior(p1, 'legend', false);
% p1=plot(ADP.t,-1*ones(size(ADP.t)),'--','LineWidth',2);
% set(p1,'Color',get(p(2),'Color'),'LineWidth',2)
% hasbehavior(p1, 'legend', false);
% p1=plot(ADP.t,2*ones(size(ADP.t)),'--','LineWidth',2);
% set(p1,'Color',get(p(3),'Color'),'LineWidth',2)
% hasbehavior(p1, 'legend', false);
% p1=plot(ADP.t,-2*ones(size(ADP.t)),'--','LineWidth',2);
% set(p1,'Color',get(p(4),'Color'),'LineWidth',2)
% hasbehavior(p1, 'legend', false);
% l=legend('$\hat{x}_{1}$','$\hat{x}_{2}$','interpreter','latex')
% set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPosition', [0 0 5 2]);
% grid on 
% print -dpng Time_Vs_TStates_MBRL_SE_Optimal_robot.png
% print -dpng Time_Vs_TStates_MBRL_SE_Optimal_robot.pdf

% State error trajectories of the x-coordinates
figure(4)
p=plot(ADP.t,(ADP.z(:,1:ADP.P.n)-ADP.z(:,ADP.P.n+1:2*ADP.P.n,1)),'Linewidth',1);
mrk1={'s','v','o','*','^','x','+','.','d','h'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,2000,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('$t$','interpreter','latex')
ylabel({'$\tilde{x}(t)$'},'interpreter','latex')
ylim([-0.2 0.2])
xlim([0 0.2])
set(gca,'FontSize',25)
l=legend('$\tilde{x}_{1}$','$\tilde{x}_{2}$','$\tilde{x}_{3}$','$\tilde{x}_{4}$','interpreter','latex')
set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
grid on
% print -dpdf Time_Vs_States_Errors_MBRL_SE_Optimal_robot.png
% print -dpdf Time_Vs_States_Errors_MBRL_SE_Optimal_robot.pdf


% Trajectory of the Critic weights 
figure(5)
p=plot(ADP.t,((ADP.z(:,2*ADP.P.n+ADP.P.d+ADP.P.W+1:2*ADP.P.n+ADP.P.d+2*ADP.P.W))),'Linewidth',1);
mrk1={'s','v','o','*','^','x','+','.','d','h'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('$t$','interpreter','latex');
ylabel('$\hat{W}_{c}(t)$','interpreter','latex');
xlim([0 70])
legend('$\hat{W}_{c_{1}}$','$\hat{W}_{c_{2}}$','$\hat{W}_{c_{3}}$','$\hat{W}_{c_{4}}$','$\hat{W}_{c_{5}}$','$\hat{W}_{c_{6}}$','$\hat{W}_{c_{7}}$','$\hat{W}_{c_{8}}$','$\hat{W}_{c_{9}}$','$\hat{W}_{c_{10}}$','interpreter','latex','NumColumns',3)
l= set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gca,'FontSize',25)
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
drawnow; 
grid on
% print -dpdf Time_Vs_States_Weights_MBRL_SE_Optimal_robot.png
% print -dpdf Time_Vs_States_Weights_MBRL_SE_Optimal_robot.pdf


% figure(7)
% p=plot(ADP.z(:,ADP.P.n+1),ADP.z(:,2*ADP.P.n,1),'v',5,'Linewidth',1);
% rectangle('position',[-7 -5 12 12])
% axis([-8 7 -7 8])
% % mrk1={'s','v','o','*','^','x','+','.','d'};
% % mrk=(mrk1(1,1:size(p,1)))';
% % set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
% % f_nummarkers(p,10,1);
% % for i=1:size(p,1)
% %     hasbehavior(p(i), 'legend', false);
% % end
% xlabel({'Estimates State, $\hat{x}_{1}$(t)'},'interpreter','latex')
% ylabel({'Estimates State, $\hat{x}_{2}$(t)'},'interpreter','latex')
% set(gca,'FontSize',10)
% l=legend('Phaseportrait, $\hat{x}$','interpreter','latex')
% set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
% set(gcf, 'PaperPositionMode', 'manual');
% set(gcf, 'PaperUnits', 'inches');
% set(gcf, 'PaperSize', [4 2]);
% set(gcf, 'PaperPosition', [0 0 5 2]);
% grid on
% print -dpng Phase_Portrait_MBRL_SE_Optimal_robot.png
% print -dpng Phase_Portrait_MBRL_SE_Optimal_robot.pdf


% figure(8)
% plot(ADP.t,((ADP.z(:,2*ADP.P.n+ADP.P.d+2*ADP.P.W+ADP.P.G^2+1))),'Linewidth',1);
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
% print -dpng Time_Vs_TStates_MBRL_SE_Optimal.png
% print -dpng Time_Vs_TStates_MBRL_SE_Optimal.pdf