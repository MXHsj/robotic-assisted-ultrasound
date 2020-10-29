%% find surface normal
clc;
clear;
close all;

dir = '/home/xihan/Myworkspace/lung_ultrasound/scripts/surface_normal.csv';
data = csvread(dir);
scale = 1;

%%
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
    pause(0.01)
end

%%
figure
stamp = 1:size(data,1)/3;
% plot x component
subplot(3,1,1)
plot(stamp,data(1:3:end-2,9),'-.','color',[0 0.4470 0.7410],'LineWidth',1)
grid on
ylabel('x comp. [m]')

% plot y component
subplot(3,1,2)
plot(stamp,data(2:3:end-1,9),'-.','color',[0.8500 0.3250 0.0980],'LineWidth',1)
grid on
ylabel('y comp. [m]')

% plot z component
subplot(3,1,3)
yyaxis left
plot(stamp,data(3:3:end,9),'-.','color',[0.9290 0.6940 0.1250],'LineWidth',1)
ylabel('z comp. [m]')
yyaxis right
plot(stamp,asind(data(3:3:end,9)),'-','color',[0.4940 0.1840 0.5560],'LineWidth',1)
grid on
xlabel('time stamp')
ylabel('z-XY angle  [deg]')

