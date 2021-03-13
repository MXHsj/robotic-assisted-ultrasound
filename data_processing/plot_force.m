%%
clc; clear; close all

curr_dir = pwd;
des_dir = 'robotic_ultrasound';
if sum(curr_dir(end-length(des_dir)+1:end) ~= des_dir)>0
    cd('robotic_ultrasound')    
end

data_raw1 = csvread('data/robot_data_log_02_24.csv');    % scan phantom
data_raw2 = csvread('data/robot_data_log_02_24_1.csv');    % scan phantom

Fz1 = data_raw1(5:end,end-1);
Fz2 = data_raw2(5:end,end-1);
contact_flag1 = data_raw1(5:end,end);
contact_flag2 = data_raw2(5:end,end);

period1S1 = 1;
period1E1 = find(contact_flag1 == 1,1);
period2S1 = find(contact_flag1(period1E1:end) == 0,1) + period1E1;
period2E1 = find(contact_flag1(period2S1:end) == 1,1) + period2S1;
period3S1 = find(contact_flag1(period2E1:end) == 0,1) + period2E1;
period3E1 = find(contact_flag1(period3S1:end) == 1,1) + period3S1;
period4S1 = find(contact_flag1(period3E1:end) == 0,1) + period3E1;
period4E1 = find(contact_flag1(period4S1:end) == 1,1) + period4S1;

period1S2 = 1;
period1E2 = find(contact_flag2 == 1,1);
period2S2 = find(contact_flag2(period1E2:end) == 0,1) + period1E2;
period2E2 = find(contact_flag2(period2S2:end) == 1,1) + period2S2;
period3S2 = find(contact_flag2(period2E2:end) == 0,1) + period2E2;
period3E2 = find(contact_flag2(period3S2:end) == 1,1) + period3S2;
period4S2 = find(contact_flag2(period3E2:end) == 0,1) + period3E2;
period4E2 = find(contact_flag2(period4S2:end) == 1,1) + period4S2;

Fz_desired = 5;
Toffset = 0;
ftime11 = period1E1:period2S1+Toffset;
ftime21 = period2E1:period3S1+Toffset;
ftime31 = period3E1:period4S1+Toffset;
ftime41 = period4E1:length(data_raw1(5:end,1:end-2))-Toffset;

ftime12 = period1E2:period2S2+Toffset;
ftime22 = period2E2:period3S2+Toffset;
ftime32 = period3E2:period4S2+Toffset;
ftime42 = period4E2:length(data_raw2(5:end,1:end-2))-Toffset;

figure
% subplot(1,4,1)
time = 1:min(length(ftime11),length(ftime12));
plot(time./10, Fz1(ftime11(1):ftime11(length(time))),'LineWidth',2);
hold on
plot(time./10, Fz2(ftime12),'LineWidth',2);
plot(time./10, Fz_desired*ones(length(time),1),'--r','LineWidth',2)

set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]','fontweight','bold','fontsize',12)
ylabel('force [N]','fontweight','bold','fontsize',12)
xlim([0 20])
ylim([-4 10])
% title('scan healthy part')
% title('scan COVID-infected part')
legend('Scan A','Scan B','desired','FontSize',18)
set(get(gca, 'XAxis'),'fontsize',16);
set(get(gca, 'YAxis'),'fontsize',16);
% axis equal
