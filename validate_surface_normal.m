%% integrated evaluation
clc;
clear;
close all;

cd ~/Myworkspace/MATLAB_WS/robotic_ultrasound
dir = 'data/reg4_validate.csv';
data_raw = csvread(dir);
data = removeBadData(data_raw);
scale = 1;

figure
stamp = 1:size(data,1)/3;

subplot(2,1,1)
est_vec = [data(1:3:end-2,10),data(2:3:end-1,10),data(3:3:end,10)];
gt_vec = [data(1:3:end-2,11),data(2:3:end-1,11),data(3:3:end,11)];
proj = dot(est_vec, gt_vec, 2);
rot_err = 1.0004 - proj;
plot(stamp,rot_err,'color',[0.4940 0.1840 0.5560],'LineWidth',1)
xlabel('time stamp')
ylabel('rotation error')
ylim([min(rot_err)-0.03, max(rot_err)+0.03])
fprintf('mean: %f \t std: %f \n', mean(rot_err), std(rot_err))

subplot(2,1,2)
est_x = data(1:3:end-2,1).*1000;
est_y = data(2:3:end-1,1).*1000;
est_z = data(3:3:end,1).*1000;
gt_x = data(1:3:end-2,12).*1000;
gt_y = data(2:3:end-1,12).*1000;
gt_z = data(3:3:end,12).*1000;
trans_err = sqrt((gt_x-est_x).^2+(gt_y-est_y).^2+(gt_z-est_z).^2);
plot(stamp,trans_err,'color',[0.3010 0.7450 0.9330],'LineWidth',1)
xlabel('time stamp')
ylabel('translation error [mm]')
ylim([min(trans_err)-1, max(trans_err)+1])
fprintf('mean: %f \t std: %f \n', mean(trans_err), std(trans_err))

%% animation
figure
for r=1:3:size(data,1)
% r = 1;
    % plot all points
    P0 = [data(r,1)*scale,data(r+1,1)*scale,data(r+2,1)];   % center
    P1 = [data(r,2)*scale,data(r+1,2)*scale,data(r+2,2)];
    P2 = [data(r,3)*scale,data(r+1,3)*scale,data(r+2,3)];
    P3 = [data(r,4)*scale,data(r+1,4)*scale,data(r+2,4)];
    P4 = [data(r,5)*scale,data(r+1,5)*scale,data(r+2,5)];
    P5 = [data(r,6)*scale,data(r+1,6)*scale,data(r+2,6)];
    P6 = [data(r,7)*scale,data(r+1,7)*scale,data(r+2,7)];
    P7 = [data(r,8)*scale,data(r+1,8)*scale,data(r+2,8)];
    P8 = [data(r,9)*scale,data(r+1,9)*scale,data(r+2,9)];
    
    Pz = [data(r,10),data(r+1,10),data(r+2,10)]+P0;
    Pz_val = [data(r,11),data(r+1,11),data(r+2,11)]+P0;
    
    X_pnt = [P0(1),P1(1),P2(1),P3(1),P4(1),P5(1),P6(1),P7(1),P8(1)];
    Y_pnt = [P0(2),P1(2),P2(2),P3(2),P4(2),P5(2),P6(2),P7(2),P8(2)];
    Z_pnt = [P0(3),P1(3),P2(3),P3(3),P4(3),P5(3),P6(3),P7(3),P8(3)];
    
    plot3(X_pnt,Y_pnt,Z_pnt,'.g','MarkerSize',20)
    hold on
    grid on
    
    % plot normal vectors
    Vz = Pz - P0;     % normal vector
    line([P0(1) Pz(1)],[P0(2) Pz(2)],[P0(3) Pz(3)], ...
        'LineWidth',1,'color',[0 0 0.8]);     % z axis
    
    Vz_val = Pz - P0;     % normal vector
    line([P0(1) Pz_val(1)],[P0(2) Pz_val(2)],[P0(3) Pz_val(3)], ...
        'LineWidth',1,'LineStyle','--','color',[0 0 0.2]);     % z axis
    
    % axis visual effects
    xlim([min(min(data(1:3:end-2,1:end-3))) max(max(data(1:3:end-2,1:end-3)))])
    ylim([min(min(data(2:3:end-1,1:end-3))) max(max(data(2:3:end-1,1:end-3)))])
    zlim([min(min(data(3:3:end-1,1:end-3))) max(max(data(3:3:end,1:end-3)))])
    set(gca,'DataAspectRatio',[1 1 1])
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    view(-30, 30)
    hold off
    drawnow
    pause(0.005)
end

%% validate rotation
figure
stamp = 1:size(data,1)/3;

% x component
subplot(3,1,1)
plot(stamp,data(1:3:end-2,10),'color',[0 0.4470 0.7410],'LineWidth',1)
grid on
hold on
plot(stamp,data(1:3:end-2,11),'-.','color',[0.4940 0.1840 0.5560],'LineWidth',1)
ylabel('x comp.')
legend('estimated','ground truth')
title('rotation')
rx_rms = rms((data(1:3:end-2,10) - data(1:3:end-2,11)));
fprintf('RMS error in x component: %f \n', rx_rms)

% y component
subplot(3,1,2)
plot(stamp,data(2:3:end-1,10),'color',[0.8500 0.3250 0.0980],'LineWidth',1)
grid on
hold on
plot(stamp,data(2:3:end-1,11),'-.','color',[0.4660 0.6740 0.1880],'LineWidth',1)
ylabel('y comp.')
legend('estimated','ground truth')
ry_rms = rms((data(2:3:end-1,10) - data(2:3:end-1,11)));
fprintf('RMS error in y component: %f \n', ry_rms)

% z component
subplot(3,1,3)
plot(stamp,data(3:3:end,10),'color',[0.9290 0.6940 0.1250],'LineWidth',1)
grid on
hold on
plot(stamp,data(3:3:end,11),'-.','color',[0.3010 0.7450 0.9330],'LineWidth',1)
ylabel('z comp.')
xlabel('time stamp')
legend('estimated','ground truth')
rz_rms = rms((data(3:3:end,10) - data(3:3:end,11)));
fprintf('RMS error in z component: %f \n', rz_rms)

%% validate translation
figure
stamp = 1:size(data,1)/3;

% x direction
subplot(3,1,1)
plot(stamp,data(1:3:end-2,1).*1000,'color',[0 0.4470 0.7410],'LineWidth',1)
grid on
hold on
plot(stamp,data(1:3:end-2,12).*1000,'-.','color',[0.4940 0.1840 0.5560],'LineWidth',1)
ylabel('x diretion [mm]')
legend('estimated','ground truth')
title('translation')
tx_rms = rms((data(1:3:end-2,1) - data(1:3:end-2,12)).*1000);
fprintf('RMS error in x diretion: %f [mm]\n', tx_rms)

% plot y component
subplot(3,1,2)
plot(stamp,data(2:3:end-1,1).*1000,'color',[0.8500 0.3250 0.0980],'LineWidth',1)
grid on
hold on
plot(stamp,data(2:3:end-1,12).*1000,'-.','color',[0.4660 0.6740 0.1880],'LineWidth',1)
ylabel('y direction [mm]')
legend('estimated','ground truth')
ty_rms = rms((data(2:3:end-1,1) - data(2:3:end-1,12)).*1000);
fprintf('RMS error in y diretion: %f [mm]\n', ty_rms)

% plot z component
subplot(3,1,3)
plot(stamp,data(3:3:end,1).*1000,'color',[0.9290 0.6940 0.1250],'LineWidth',1)
grid on
hold on
plot(stamp,data(3:3:end,12).*1000,'-.','color',[0.3010 0.7450 0.9330],'LineWidth',1)
ylabel('z direction [mm]')
xlabel('time stamp')
legend('estimated','ground truth')
tz_rms = rms((data(3:3:end,1) - data(3:3:end,12)).*1000);
fprintf('RMS error in z diretion: %f [mm]\n', tz_rms)
