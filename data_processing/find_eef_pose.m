%% read data
clc;
clear;
close all;

cd ~/Myworkspace/MATLAB_WS/robotic_ultrasound
% dir = 'data/surface_normal_test.csv';
dir = 'data/L1_kernel14.csv';
data_raw = csvread(dir);
data = removeBadData(data_raw);
scale = 1;

%% points & axis animation
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
    
    X_pnt = [P0(1),P1(1),P2(1),P3(1),P4(1),P5(1),P6(1),P7(1),P8(1)];
    Y_pnt = [P0(2),P1(2),P2(2),P3(2),P4(2),P5(2),P6(2),P7(2),P8(2)];
    Z_pnt = [P0(3),P1(3),P2(3),P3(3),P4(3),P5(3),P6(3),P7(3),P8(3)];
    
    plot3(X_pnt,Y_pnt,Z_pnt,'.g','MarkerSize',20)
    hold on
    grid on
    
    % target tip frame
    Vz = Pz - P0;     % normal vector
    line([P0(1) Pz(1)],[P0(2) Pz(2)],[P0(3) Pz(3)], ...
        'LineWidth',2,'color',[0 0 1]);     % z axis
    
    xx = 1.0;
    yx = 0;
    zx = -(Vz(2)*(yx-P0(2))+Vz(1)*(xx-P0(1)))/Vz(3)+P0(3);
    Vx = [xx,yx,zx] - P0;
    Vx = Vx/norm(Vx);
    line([P0(1) xx],[P0(2) yx],[P0(3) zx], ...
        'LineWidth',2,'color',[1 0 0]);     % x axis
    
    Vy = cross(Vz, Vx);
    Vy = Vy/norm(Vy);
    line([P0(1) Vy(1)+P0(1)],[P0(2) Vy(2)+P0(2)],[P0(3) Vy(3)+P0(3)], ...
        'LineWidth',2,'color',[0 1 0]);     % y axis
    
    % axis visual effects
    xlim([min(min(data(1:3:end-2,1:end-1))) max(max(data(1:3:end-2,1:end-1)))])
    ylim([min(min(data(2:3:end-1,1:end-1))) max(max(data(2:3:end-1,1:end-1)))])
    zlim([min(min(data(3:3:end,1:end-1))) max(max(data(3:3:end,1:end-1)))])
    set(gca,'DataAspectRatio',[1 1 1])
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    
    Rtip = [Vx',Vy',Vz'];
    Ttip = [[Rtip, P0'];[0 0 0 1]];
    disp(Ttip)
    
    hold off
    drawnow
    pause(0.004)
end

%% plot data
stamp = 1:size(data,1)/3;
% rotation
figure
subplot(3,1,1)
plot(stamp,data(1:3:end-2,10),'-','color',[0.9290 0.6940 0.1250],'LineWidth',1)
ylabel('x comp.')
title('rotation')
grid on
subplot(3,1,2)
plot(stamp,data(2:3:end-1,10),'-','color',[0.4940 0.1840 0.5560],'LineWidth',1)
ylabel('y comp.')
grid on
subplot(3,1,3)
plot(stamp,data(3:3:end,10),'-','color',[0 0.4470 0.7410],'LineWidth',1)
xlabel('time stamp')
ylabel('z comp.')
grid on

% translation
figure
subplot(3,1,1)
plot(stamp,data(1:3:end-2,1).*1000,'-','color',[0.9290 0.6940 0.1250],'LineWidth',1)
ylabel('x dir. [mm]')
title('translation')
grid on
subplot(3,1,2)
plot(stamp,data(2:3:end-1,1).*1000,'-','color',[0.4940 0.1840 0.5560],'LineWidth',1)
ylabel('y dir. [mm]')
grid on
subplot(3,1,3)
plot(stamp,data(3:3:end,1).*1000,'-','color',[0 0.4470 0.7410],'LineWidth',1)
xlabel('time stamp')
ylabel('z dir. [mm]')
grid on


%% stability analysis
figure
stamp = 1:size(data,1)/3;
[az,el,r] = cart2sph(data(1:3:end-2,10),data(2:3:end-1,10),data(3:3:end,10));

% plot z component
subplot(2,1,2)
% yyaxis left
% plot(stamp,data(3:3:end,10),'-.','color',[0.9290 0.6940 0.1250],'LineWidth',1)
% ylabel('[mm]')
% yyaxis right
theta = 90 - asind(data(3:3:end,10));
plot(stamp,theta,'-','color',[0.4940 0.1840 0.5560],'LineWidth',1)
grid on
xlabel('time stamp')
ylabel('theta [deg]')
legend('azimuthal angle')

% plot x component
subplot(2,1,1)
% yyaxis left
% plot(stamp,data(1:3:end-2,10),'-.','color',[0 0.4470 0.7410],'LineWidth',1)
% ylabel('[mm]')
% yyaxis right
d = cosd(theta);
phi = acosd(data(1:3:end-2,10)./d);
plot(stamp,phi, '-','color',[0.4660 0.6740 0.1880],'LineWidth',1)
grid on
legend('polar angle')
ylabel('phi  [deg]')