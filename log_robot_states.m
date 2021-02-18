%% robot data analysis
clc; clear; close all

data_raw = csvread('data/robot_data_log_02_17.csv');

contact_flag = data_raw(5:end,end);
Fz = data_raw(5:end,end-1);

target_flat = data_raw(1:4,1:end-2);
target1 = reshape(target_flat(1,:),4,4)';
target2 = reshape(target_flat(2,:),4,4)';
target3 = reshape(target_flat(3,:),4,4)';
target4 = reshape(target_flat(4,:),4,4)';
tar1_rpy = rotm2eul(target1(1:3,1:3))';
tar1_xyz = target1(1:3,4).*1000;
tar2_rpy = rotm2eul(target2(1:3,1:3))';
tar2_xyz = target2(1:3,4).*1000;
tar3_rpy = rotm2eul(target3(1:3,1:3))';
tar3_xyz = target3(1:3,4).*1000;
tar4_rpy = rotm2eul(target4(1:3,1:3))';
tar4_xyz = target4(1:3,4).*1000;

pose_flat = data_raw(5:end,1:end-2);
pos_rpy = -1.*ones(length(pose_flat),3);
pos_xyz = -1.*ones(length(pose_flat),3);
for i = 1:length(pose_flat)
    pose = reshape(pose_flat(i,:),4,4)';
    pos_rpy(i,:) = rotm2eul(pose(1:3,1:3));
    pos_xyz(i,:) = pose(1:3,4).*1000;
end

% angle wrap around
if tar1_rpy(3) < 0 
    tar1_rpy(3) = tar1_rpy(3) + 2*pi;
end
if tar2_rpy(3) < 0 
    tar2_rpy(3) = tar2_rpy(3) + 2*pi;
end
if tar3_rpy(3) < 0 
    tar3_rpy(3) = tar3_rpy(3) + 2*pi;
end
if tar4_rpy(3) < 0 
    tar4_rpy(3) = tar4_rpy(3) + 2*pi;
end
pos_rpy(pos_rpy(:,3)<0,3) = pos_rpy(pos_rpy(:,3)<0,3) + 2*pi;

Fz_desired = 5;

period1S = 1;
period1E = find(contact_flag == 1,1);
period2S = find(contact_flag(period1E:end) == 0,1) + period1E;
period2E = find(contact_flag(period2S:end) == 1,1) + period2S;
period3S = find(contact_flag(period2E:end) == 0,1) + period2E;
period3E = find(contact_flag(period3S:end) == 1,1) + period3S;
period4S = find(contact_flag(period3E:end) == 0,1) + period3E;
period4E = find(contact_flag(period4S:end) == 1,1) + period4S;

time1 = period1S:period2S;
time2 = period2S:period3S;
time3 = period3S:period4S;
time4 = period4S:length(pose_flat);

%% plot translation
figure
subplot(2,2,1)
plot(time1./10,tar1_xyz*ones(1,length(time1))-pos_xyz(time1,:)','LineWidth',1.5)
% hold on
% plot(time1,tar1_xyz*ones(1,length(time1)),'--')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('home to target1')
legend('x','y','z','Location','southeast','NumColumns',1)

subplot(2,2,2)
plot(time2./10,tar2_xyz*ones(1,length(time2))-pos_xyz(time2,:)','LineWidth',1.5)
% hold on
% plot(time2,tar2_xyz*ones(1,length(time2)),'--')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('target1 to target2')
legend('x','y','z','Location','southeast','NumColumns',1)

subplot(2,2,3)
plot(time3./10,tar3_xyz*ones(1,length(time3))-pos_xyz(time3,:)','LineWidth',1.5)
% hold on
% plot(time3,tar3_xyz*ones(1,length(time3)),'--')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('target2 to target3')
legend('x','y','z','Location','southeast','NumColumns',1)

subplot(2,2,4)
plot(time4./10,tar4_xyz*ones(1,length(time4))-pos_xyz(time4,:)','LineWidth',1.5)
% hold on
% plot(time4,tar4_xyz*ones(1,length(time4)),'--')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('target3 to target4')
legend('x','y','z','Location','southeast','NumColumns',1)
set(gcf, 'Position',  [500, 500, 500, 500])

%% plot rotation
figure
subplot(2,2,1)
plot(time1./10,tar1_rpy*ones(1,length(time1))-pos_rpy(time1,:)','LineWidth',1.5)
% hold on
% plot(time1,tar1_xyz*ones(1,length(time1)),'--')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('home to target1')
legend('row','pitch','yaw','Location','southeast','NumColumns',1)

subplot(2,2,2)
plot(time2./10,tar2_rpy*ones(1,length(time2))-pos_rpy(time2,:)','LineWidth',1.5)
% hold on
% plot(time2,tar2_xyz*ones(1,length(time2)),'--')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('target1 to target2')
legend('row','pitch','yaw','Location','northeast','NumColumns',1)

subplot(2,2,3)
plot(time3./10,tar3_rpy*ones(1,length(time3))-pos_rpy(time3,:)','LineWidth',1.5)
% hold on
% plot(time3,tar3_xyz*ones(1,length(time3)),'--')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('target2 to target3')
legend('row','pitch','yaw','Location','southeast','NumColumns',1)

subplot(2,2,4)
plot(time4./10,tar4_rpy*ones(1,length(time4))-pos_rpy(time4,:)','LineWidth',1.5)
% hold on
% plot(time4,tar4_xyz*ones(1,length(time4)),'--')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('target3 to target4')
legend('row','pitch','yaw','Location','northeast','NumColumns',1)
set(gcf, 'Position',  [500, 500, 500, 500])


%% plot force
Toffset = 80;
ftime1 = period1E:period2S+Toffset;
ftime2 = period2E:period3S+Toffset;
ftime3 = period3E:period4S+Toffset;
ftime4 = period4E:length(pose_flat)-Toffset;

figure
subplot(2,2,1)
plot(ftime1./10, Fz(ftime1),'LineWidth',1.5);
hold on
plot(ftime1./10, Fz_desired*ones(length(ftime1),1),'--r')
xlabel('time [seconds]')
ylabel('force Z [N]')
title('scan target1')
axis equal

subplot(2,2,2)
plot(ftime2./10, Fz(ftime2),'LineWidth',1.5);
hold on
plot(ftime2./10, Fz_desired*ones(length(ftime2),1),'--r')
xlabel('time [seconds]')
ylabel('force Z [N]')
title('scan target2')
axis equal

subplot(2,2,3)
plot(ftime3./10, Fz(ftime3),'LineWidth',1.5);
hold on
plot(ftime3./10, Fz_desired*ones(length(ftime3),1),'--r')
xlabel('time [seconds]')
ylabel('force Z [N]')
title('scan target3')
axis equal

subplot(2,2,4)
plot(ftime4./10, Fz(ftime4),'LineWidth',1.5);
hold on
plot(ftime4./10, Fz_desired*ones(length(ftime4),1),'--r')
xlabel('time [seconds]')
ylabel('force Z [N]')
title('scan target4')
axis equal
set(gcf, 'Position',  [500, 500, 500, 500])