%% force animation for IROS video
clc
clear
close all

force_rec = csvread('data/force_data_log_03_07.csv');
force_des = 5;

freq = 20;
time = 1:length(force_rec);
figure
plot(time./freq, force_rec(time,end))
hold on
plot(time./freq, force_des*ones(1,length(time)),'-r','LineWidth',1)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
xlim([0,122])
ylabel('force [N]')
ylim([-1, 8])
legend('recorded','desired','Location','northeast','FontSize',13)