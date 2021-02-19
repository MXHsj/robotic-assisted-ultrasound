%% robot data analysis
clc; clear; close all

data_raw = csvread('data/robot_data_log_02_17.csv');

contact_flag = data_raw(5:end,end);
Fz = data_raw(5:end,end-1);

target_flat = data_raw(1:4,1:end-2);
entry1 = reshape(target_flat(1,:),4,4)';
entry2 = reshape(target_flat(2,:),4,4)';
entry3 = reshape(target_flat(3,:),4,4)';
entry4 = reshape(target_flat(4,:),4,4)';
ent1_rpy = rotm2eul(entry1(1:3,1:3))';
ent1_xyz = entry1(1:3,4).*1000;
ent2_rpy = rotm2eul(entry2(1:3,1:3))';
ent2_xyz = entry2(1:3,4).*1000;
ent3_rpy = rotm2eul(entry3(1:3,1:3))';
ent3_xyz = entry3(1:3,4).*1000;
ent4_rpy = rotm2eul(entry4(1:3,1:3))';
ent4_xyz = entry4(1:3,4).*1000;

Cd = 0.075; % entry pose
tar1_xyz = ent1_xyz + Cd*1000*entry1(1:3,3);
tar2_xyz = ent2_xyz + Cd*1000*entry2(1:3,3);
tar3_xyz = ent3_xyz + Cd*1000*entry3(1:3,3);
tar4_xyz = ent4_xyz + Cd*1000*entry4(1:3,3);
tar1_rpy = ent1_rpy;
tar2_rpy = ent2_rpy;
tar3_rpy = ent3_rpy;
tar4_rpy = ent4_rpy;

pose_flat = data_raw(5:end,1:end-2);
pos_rpy = -1.*ones(length(pose_flat),3);
pos_xyz = -1.*ones(length(pose_flat),3);
for i = 1:length(pose_flat)
    pose = reshape(pose_flat(i,:),4,4)';
    pos_rpy(i,:) = rotm2eul(pose(1:3,1:3));
    pos_xyz(i,:) = pose(1:3,4).*1000;
end

% angle wrap around
ent1_rpy(3) = wrappedAngle(ent1_rpy(3));
ent2_rpy(3) = wrappedAngle(ent2_rpy(3));
ent3_rpy(3) = wrappedAngle(ent3_rpy(3));
ent4_rpy(3) = wrappedAngle(ent4_rpy(3));
tar1_rpy(3) = wrappedAngle(tar1_rpy(3));
tar2_rpy(3) = wrappedAngle(tar2_rpy(3));
tar3_rpy(3) = wrappedAngle(tar3_rpy(3));
tar4_rpy(3) = wrappedAngle(tar4_rpy(3));
pos_rpy(pos_rpy(:,3)<0,3) = pos_rpy(pos_rpy(:,3)<0,3) + 2*pi;

period1S = 1;
period1E = find(contact_flag == 1,1);
period2S = find(contact_flag(period1E:end) == 0,1) + period1E;
period2E = find(contact_flag(period2S:end) == 1,1) + period2S;
period3S = find(contact_flag(period2E:end) == 0,1) + period2E;
period3E = find(contact_flag(period3S:end) == 1,1) + period3S;
period4S = find(contact_flag(period3E:end) == 0,1) + period3E;
period4E = find(contact_flag(period4S:end) == 1,1) + period4S;

time1 = period1S:period1E;
time2 = period1E:period2S;
time3 = period2S:period2E;
time4 = period2E:period3S;
time5 = period3S:period3E;
time6 = period3E:period4S;
time7 = period4S:period4E;
time8 = period4E:length(pose_flat);

%% plot translation
figure
subplot(4,2,1)
plot(time1./10,ent1_xyz*ones(1,length(time1))-pos_xyz(time1,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('home to entry1')
legend('x','y','z','Location','northeast','NumColumns',1)

subplot(4,2,2)
plot(time2./10,tar1_xyz*ones(1,length(time2))-pos_xyz(time2,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('entry1 to target1')
legend('x','y','z','Location','southeast','NumColumns',1)

subplot(4,2,3)
plot(time3./10,ent2_xyz*ones(1,length(time3))-pos_xyz(time3,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('target1 to target2')
legend('x','y','z','Location','southeast','NumColumns',1)

subplot(4,2,4)
plot(time4./10,tar2_xyz*ones(1,length(time4))-pos_xyz(time4,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('entry2 to target2')
legend('x','y','z','Location','southeast','NumColumns',1)

subplot(4,2,5)
plot(time5./10,ent3_xyz*ones(1,length(time5))-pos_xyz(time5,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('target2 to target3')
legend('x','y','z','Location','southeast','NumColumns',1)

subplot(4,2,6)
plot(time6./10,tar3_xyz*ones(1,length(time6))-pos_xyz(time6,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('entry3 to target3')
legend('x','y','z','Location','southeast','NumColumns',1)

subplot(4,2,7)
plot(time7./10,ent4_xyz*ones(1,length(time7))-pos_xyz(time7,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('target3 to target4')
legend('x','y','z','Location','southeast','NumColumns',1)

subplot(4,2,8)
plot(time8./10,tar4_xyz*ones(1,length(time8))-pos_xyz(time8,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('translation error [mm]')
title('entry4 to target4')
legend('x','y','z','Location','southeast','NumColumns',1)

set(gcf, 'Position',  [500, 100, 500, 850])

%% plot rotation
figure
subplot(4,2,1)
plot(time1./10,ent1_rpy*ones(1,length(time1))-pos_rpy(time1,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('home to entry1')
legend('row','pitch','yaw','Location','southeast','NumColumns',1)

subplot(4,2,2)
plot(time2./10,tar1_rpy*ones(1,length(time2))-pos_rpy(time2,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('entry1 to target1')
legend('row','pitch','yaw','Location','northwest','NumColumns',1)

subplot(4,2,3)
plot(time3./10,ent2_rpy*ones(1,length(time3))-pos_rpy(time3,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('target1 to target2')
legend('row','pitch','yaw','Location','northeast','NumColumns',1)

subplot(4,2,4)
plot(time4./10,tar2_rpy*ones(1,length(time4))-pos_rpy(time4,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('entry1 to target1')
legend('row','pitch','yaw','Location','west','NumColumns',1)

subplot(4,2,5)
plot(time5./10,ent3_rpy*ones(1,length(time5))-pos_rpy(time5,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('target2 to target3')
legend('row','pitch','yaw','Location','southeast','NumColumns',1)

subplot(4,2,6)
plot(time6./10,tar3_rpy*ones(1,length(time6))-pos_rpy(time6,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('entry1 to target1')
legend('row','pitch','yaw','Location','west','NumColumns',1)

subplot(4,2,7)
plot(time7./10,ent4_rpy*ones(1,length(time7))-pos_rpy(time7,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('target3 to target4')
legend('row','pitch','yaw','Location','northeast','NumColumns',1)

subplot(4,2,8)
plot(time8./10,tar4_rpy*ones(1,length(time8))-pos_rpy(time8,:)','LineWidth',1.5)
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('rotation error [rad]')
title('entry1 to target1')
legend('row','pitch','yaw','Location','northeast','NumColumns',1)

set(gcf, 'Position',  [500, 100, 500, 850])


%% plot force
Fz_desired = 5;
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
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('force [N]')
title('scan target1')
legend('recorded','desired')
axis equal

subplot(2,2,2)
plot(ftime2./10, Fz(ftime2),'LineWidth',1.5);
hold on
plot(ftime2./10, Fz_desired*ones(length(ftime2),1),'--r')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('force [N]')
title('scan target2')
legend('recorded','desired')
axis equal

subplot(2,2,3)
plot(ftime3./10, Fz(ftime3),'LineWidth',1.5);
hold on
plot(ftime3./10, Fz_desired*ones(length(ftime3),1),'--r')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('force [N]')
title('scan target3')
legend('recorded','desired')
axis equal

subplot(2,2,4)
plot(ftime4./10, Fz(ftime4),'LineWidth',1.5);
hold on
plot(ftime4./10, Fz_desired*ones(length(ftime4),1),'--r')
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('time [seconds]')
ylabel('force [N]')
title('scan target4')
legend('recorded','desired')
axis equal
set(gcf, 'Position',  [500, 500, 500, 500])