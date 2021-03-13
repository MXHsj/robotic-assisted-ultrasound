%% read data
clc; clear; close all;
cd ~/Myworkspace/MATLAB_WS/robotic_ultrasound

L1_kernel4_raw = csvread('data/L1_kernel4.csv');
L1_kernel6_raw = csvread('data/L1_kernel6.csv');
L1_kernel8_raw = csvread('data/L1_kernel8.csv');
L1_kernel10_raw = csvread('data/L1_kernel10.csv');
L1_kernel12_raw = csvread('data/L1_kernel12.csv');
L1_kernel14_raw = csvread('data/L1_kernel14.csv');
L1_kernel16_raw = csvread('data/L1_kernel16.csv');
L1_kernel18_raw = csvread('data/L1_kernel18.csv');

L1_kernel4 = removeBadData(L1_kernel4_raw);
L1_kernel6 = removeBadData(L1_kernel6_raw);
L1_kernel8 = removeBadData(L1_kernel8_raw);
L1_kernel10 = removeBadData(L1_kernel10_raw);
L1_kernel12 = removeBadData(L1_kernel12_raw);
L1_kernel14 = removeBadData(L1_kernel14_raw);
L1_kernel16 = removeBadData(L1_kernel16_raw);
L1_kernel18 = removeBadData(L1_kernel18_raw);

L2_kernel4_raw = csvread('data/L2_kernel4.csv');
L2_kernel6_raw = csvread('data/L2_kernel6.csv');
L2_kernel8_raw = csvread('data/L2_kernel8.csv');
L2_kernel10_raw = csvread('data/L2_kernel10.csv');
L2_kernel12_raw = csvread('data/L2_kernel12.csv');
L2_kernel14_raw = csvread('data/L2_kernel14.csv');
L2_kernel16_raw = csvread('data/L2_kernel16.csv');
L2_kernel18_raw = csvread('data/L2_kernel18.csv');

L2_kernel4 = removeBadData(L2_kernel4_raw);
L2_kernel6 = removeBadData(L2_kernel6_raw);
L2_kernel8 = removeBadData(L2_kernel8_raw);
L2_kernel10 = removeBadData(L2_kernel10_raw);
L2_kernel12 = removeBadData(L2_kernel12_raw);
L2_kernel14 = removeBadData(L2_kernel14_raw);
L2_kernel16 = removeBadData(L2_kernel16_raw);
L2_kernel18 = removeBadData(L2_kernel18_raw);

start = 1;
sample_size = 1100;
 
%% translational stability analysis
% --------------------------- L1 ------------------------------
L1_kernel4_tx = std(L1_kernel4(start:3:sample_size-2,1));
L1_kernel4_ty = std(L1_kernel4(start+1:3:sample_size-1,1));
L1_kernel4_tz = std(L1_kernel4(start+2:3:sample_size,1));

L1_kernel6_tx = std(L1_kernel6(start:3:sample_size-2,1));
L1_kernel6_ty = std(L1_kernel6(start+1:3:sample_size-1,1));
L1_kernel6_tz = std(L1_kernel6(start+2:3:sample_size,1));

L1_kernel8_tx = std(L1_kernel8(start:3:sample_size-2,1));
L1_kernel8_ty = std(L1_kernel8(start+1:3:sample_size-1,1));
L1_kernel8_tz = std(L1_kernel8(start+2:3:sample_size,1));

L1_kernel10_tx = std(L1_kernel10(start:3:sample_size-2,1));
L1_kernel10_ty = std(L1_kernel10(start+1:3:sample_size-1,1));
L1_kernel10_tz = std(L1_kernel10(start+2:3:sample_size,1));

L1_kernel12_tx = std(L1_kernel12(start:3:sample_size-2,1));
L1_kernel12_ty = std(L1_kernel12(start+1:3:sample_size-1,1));
L1_kernel12_tz = std(L1_kernel12(start+2:3:sample_size,1));

L1_kernel14_tx = std(L1_kernel14(start:3:sample_size-2,1));
L1_kernel14_ty = std(L1_kernel14(start+1:3:sample_size-1,1));
L1_kernel14_tz = std(L1_kernel14(start+2:3:sample_size,1));

L1_kernel16_tx = std(L1_kernel16(start:3:sample_size-2,1));
L1_kernel16_ty = std(L1_kernel16(start+1:3:sample_size-1,1));
L1_kernel16_tz = std(L1_kernel16(start+2:3:sample_size,1));

L1_kernel18_tx = std(L1_kernel18(start:3:sample_size-2,1));
L1_kernel18_ty = std(L1_kernel18(start+1:3:sample_size-1,1));
L1_kernel18_tz = std(L1_kernel18(start+2:3:sample_size,1));

% --------------------------- L2 ------------------------------
L2_kernel4_tx = std(L2_kernel4(start:3:sample_size-2,1));
L2_kernel4_ty = std(L2_kernel4(start+1:3:sample_size-1,1));
L2_kernel4_tz = std(L2_kernel4(start+2:3:sample_size,1));

L2_kernel6_tx = std(L2_kernel6(start:3:sample_size-2,1));
L2_kernel6_ty = std(L2_kernel6(start+1:3:sample_size-1,1));
L2_kernel6_tz = std(L2_kernel6(start+2:3:sample_size,1));

L2_kernel8_tx = std(L2_kernel8(start:3:sample_size-2,1));
L2_kernel8_ty = std(L2_kernel8(start+1:3:sample_size-1,1));
L2_kernel8_tz = std(L2_kernel8(start+2:3:sample_size,1));

L2_kernel10_tx = std(L2_kernel10(start:3:sample_size-2,1));
L2_kernel10_ty = std(L2_kernel10(start+1:3:sample_size-1,1));
L2_kernel10_tz = std(L2_kernel10(start+2:3:sample_size,1));

L2_kernel12_tx = std(L2_kernel12(start:3:sample_size-2,1));
L2_kernel12_ty = std(L2_kernel12(start+1:3:sample_size-1,1));
L2_kernel12_tz = std(L2_kernel12(start+2:3:sample_size,1));

L2_kernel14_tx = std(L2_kernel14(start:3:sample_size-2,1));
L2_kernel14_ty = std(L2_kernel14(start+1:3:sample_size-1,1));
L2_kernel14_tz = std(L2_kernel14(start+2:3:sample_size,1));

L2_kernel16_tx = std(L2_kernel16(start:3:sample_size-2,1));
L2_kernel16_ty = std(L2_kernel16(start+1:3:sample_size-1,1));
L2_kernel16_tz = std(L2_kernel16(start+2:3:sample_size,1));

L2_kernel18_tx = std(L2_kernel18(start:3:sample_size-2,1));
L2_kernel18_ty = std(L2_kernel18(start+1:3:sample_size-1,1));
L2_kernel18_tz = std(L2_kernel18(start+2:3:sample_size,1));

kernels_t = 1000.* ...
    [mean([L1_kernel4_tx,L2_kernel4_tx]), mean([L1_kernel4_ty,L2_kernel4_ty]), mean([L1_kernel4_tz,L2_kernel4_tz]);
     mean([L1_kernel6_tx,L2_kernel6_tx]), mean([L1_kernel6_ty,L2_kernel6_ty]), mean([L1_kernel6_tz,L2_kernel6_tz]);
     mean([L1_kernel8_tx,L2_kernel8_tx]), mean([L1_kernel8_ty,L2_kernel8_ty]), mean([L1_kernel8_tz,L2_kernel8_tz]);
     mean([L1_kernel10_tx,L2_kernel10_tx]), mean([L1_kernel10_ty,L2_kernel10_ty]), mean([L1_kernel10_tz,L2_kernel10_tz]);
     mean([L1_kernel12_tx,L2_kernel12_tx]), mean([L1_kernel12_ty,L2_kernel12_ty]), mean([L1_kernel12_tz,L2_kernel12_tz]);
     mean([L1_kernel14_tx,L2_kernel14_tx]), mean([L1_kernel14_ty,L2_kernel14_ty]), mean([L1_kernel14_tz,L2_kernel14_tz]);
     mean([L1_kernel16_tx,L2_kernel16_tx]), mean([L1_kernel16_ty,L2_kernel16_ty]), mean([L1_kernel16_tz,L2_kernel16_tz]);
     mean([L1_kernel18_tx,L2_kernel18_tx]), mean([L1_kernel18_ty,L2_kernel18_ty]), mean([L1_kernel18_tz,L2_kernel18_tz])];
       
x_axis = 4:2:18;
figure
trans_chart = bar(x_axis(2:end), kernels_t(2:end,:));
err_t = 1.* ...
    [abs(L1_kernel4_tx-L2_kernel4_tx), abs(L1_kernel4_ty-L2_kernel4_ty), abs(L1_kernel4_tz-L2_kernel4_tz);
     abs(L1_kernel6_tx-L2_kernel6_tx), abs(L1_kernel6_ty-L2_kernel6_ty), abs(L1_kernel6_tz-L2_kernel6_tz);
     abs(L1_kernel8_tx-L2_kernel8_tx), abs(L1_kernel8_ty-L2_kernel8_ty), abs(L1_kernel8_tz-L2_kernel8_tz);
     abs(L1_kernel10_tx-L2_kernel10_tx), abs(L1_kernel10_ty-L2_kernel10_ty), abs(L1_kernel10_tz-L2_kernel10_tz);
     abs(L1_kernel12_tx-L2_kernel12_tx), abs(L1_kernel12_ty-L2_kernel12_ty), abs(L1_kernel12_tz-L2_kernel12_tz);
     abs(L1_kernel14_tx-L2_kernel14_tx), abs(L1_kernel14_ty-L2_kernel14_ty), abs(L1_kernel14_tz-L2_kernel14_tz);
     abs(L1_kernel16_tx-L2_kernel16_tx), abs(L1_kernel16_ty-L2_kernel16_ty), abs(L1_kernel16_tz-L2_kernel16_tz);
     abs(L1_kernel18_tx-L2_kernel18_tx), abs(L1_kernel18_ty-L2_kernel18_ty), abs(L1_kernel18_tz-L2_kernel18_tz)];
tx = [];
for i = 1:3
    tx = [tx ; trans_chart(i).XEndPoints];
end
hold on
errorbar(tx', kernels_t(2:end,:), err_t(2:end,:))
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('kernal size [pixel]')
ylabel('standard deviation [mm]')
ylim([0, 0.65])
legend('x diretion','y direction','z direction')
title('kernel size vs. translational stability')

%% rotational stability analysis
% --------------------------- L1 ------------------------------
L1_kernel4_rx = std(L1_kernel4(start:3:sample_size-2,10));
L1_kernel4_ry = std(L1_kernel4(start+1:3:sample_size-1,10));
L1_kernel4_rz = std(L1_kernel4(start+2:3:sample_size,10));

L1_kernel6_rx = std(L1_kernel6(start:3:sample_size-2,10));
L1_kernel6_ry = std(L1_kernel6(start+1:3:sample_size-1,10));
L1_kernel6_rz = std(L1_kernel6(start+2:3:sample_size,10));

L1_kernel8_rx = std(L1_kernel8(start:3:sample_size-2,10));
L1_kernel8_ry = std(L1_kernel8(start+1:3:sample_size-1,10));
L1_kernel8_rz = std(L1_kernel8(start+2:3:sample_size,10));

L1_kernel10_rx = std(L1_kernel10(start:3:sample_size-2,10));
L1_kernel10_ry = std(L1_kernel10(start+1:3:sample_size-1,10));
L1_kernel10_rz = std(L1_kernel10(start+2:3:sample_size,10));

L1_kernel12_rx = std(L1_kernel12(start:3:sample_size-2,10));
L1_kernel12_ry = std(L1_kernel12(start+1:3:sample_size-1,10));
L1_kernel12_rz = std(L1_kernel12(start+2:3:sample_size,10));

L1_kernel14_rx = std(L1_kernel14(start:3:sample_size-2,10));
L1_kernel14_ry = std(L1_kernel14(start+1:3:sample_size-1,10));
L1_kernel14_rz = std(L1_kernel14(start+2:3:sample_size,10));

L1_kernel16_rx = std(L1_kernel16(start:3:sample_size-2,10));
L1_kernel16_ry = std(L1_kernel16(start+1:3:sample_size-1,10));
L1_kernel16_rz = std(L1_kernel16(start+2:3:sample_size,10));

L1_kernel18_rx = std(L1_kernel18(start:3:sample_size-2,10));
L1_kernel18_ry = std(L1_kernel18(start+1:3:sample_size-1,10));
L1_kernel18_rz = std(L1_kernel18(start+2:3:sample_size,10));

% --------------------------- L1 ------------------------------
L2_kernel4_rx = std(L2_kernel4(start:3:sample_size-2,10));
L2_kernel4_ry = std(L2_kernel4(start+1:3:sample_size-1,10));
L2_kernel4_rz = std(L2_kernel4(start+2:3:sample_size,10));

L2_kernel6_rx = std(L2_kernel6(start:3:sample_size-2,10));
L2_kernel6_ry = std(L2_kernel6(start+1:3:sample_size-1,10));
L2_kernel6_rz = std(L2_kernel6(start+2:3:sample_size,10));

L2_kernel8_rx = std(L2_kernel8(start:3:sample_size-2,10));
L2_kernel8_ry = std(L2_kernel8(start+1:3:sample_size-1,10));
L2_kernel8_rz = std(L2_kernel8(start+2:3:sample_size,10));

L2_kernel10_rx = std(L2_kernel10(start:3:sample_size-2,10));
L2_kernel10_ry = std(L2_kernel10(start+1:3:sample_size-1,10));
L2_kernel10_rz = std(L2_kernel10(start+2:3:sample_size,10));

L2_kernel12_rx = std(L2_kernel12(start:3:sample_size-2,10));
L2_kernel12_ry = std(L2_kernel12(start+1:3:sample_size-1,10));
L2_kernel12_rz = std(L2_kernel12(start+2:3:sample_size,10));

L2_kernel14_rx = std(L2_kernel14(start:3:sample_size-2,10));
L2_kernel14_ry = std(L2_kernel14(start+1:3:sample_size-1,10));
L2_kernel14_rz = std(L2_kernel14(start+2:3:sample_size,10));

L2_kernel16_rx = std(L2_kernel16(start:3:sample_size-2,10));
L2_kernel16_ry = std(L2_kernel16(start+1:3:sample_size-1,10));
L2_kernel16_rz = std(L2_kernel16(start+2:3:sample_size,10));

L2_kernel18_rx = std(L2_kernel18(start:3:sample_size-2,10));
L2_kernel18_ry = std(L2_kernel18(start+1:3:sample_size-1,10));
L2_kernel18_rz = std(L2_kernel18(start+2:3:sample_size,10));

kernels_r = 1.* ...
    [mean([L1_kernel4_rx,L2_kernel4_rx]), mean([L1_kernel4_ry,L2_kernel4_ry]), mean([L1_kernel4_rz,L2_kernel4_rz]);
     mean([L1_kernel6_rx,L2_kernel6_rx]), mean([L1_kernel6_ry,L2_kernel6_ry]), mean([L1_kernel6_rz,L2_kernel6_rz]);
     mean([L1_kernel8_rx,L2_kernel8_rx]), mean([L1_kernel8_ry,L2_kernel8_ry]), mean([L1_kernel8_rz,L2_kernel8_rz]);
     mean([L1_kernel10_rx,L2_kernel10_rx]), mean([L1_kernel10_ry,L2_kernel10_ry]), mean([L1_kernel10_rz,L2_kernel10_rz]);
     mean([L1_kernel12_rx,L2_kernel12_rx]), mean([L1_kernel12_ry,L2_kernel12_ry]), mean([L1_kernel12_rz,L2_kernel12_rz]);
     mean([L1_kernel14_rx,L2_kernel14_rx]), mean([L1_kernel14_ry,L2_kernel14_ry]), mean([L1_kernel14_rz,L2_kernel14_rz]);
     mean([L1_kernel16_rx,L2_kernel16_rx]), mean([L1_kernel16_ry,L2_kernel16_ry]), mean([L1_kernel16_rz,L2_kernel16_rz]);
     mean([L1_kernel18_rx,L2_kernel18_rx]), mean([L1_kernel18_ry,L2_kernel18_ry]), mean([L1_kernel18_rz,L2_kernel18_rz])];
       
x_axis = 4:2:18;
err_r = 1.* ...
    [abs(L1_kernel4_rx-L2_kernel4_rx), abs(L1_kernel4_ry-L2_kernel4_ry), abs(L1_kernel4_rz-L2_kernel4_rz);
     abs(L1_kernel6_rx-L2_kernel6_rx), abs(L1_kernel6_ry-L2_kernel6_ry), abs(L1_kernel6_rz-L2_kernel6_rz);
     abs(L1_kernel8_rx-L2_kernel8_rx), abs(L1_kernel8_ry-L2_kernel8_ry), abs(L1_kernel8_rz-L2_kernel8_rz);
     abs(L1_kernel10_rx-L2_kernel10_rx), abs(L1_kernel10_ry-L2_kernel10_ry), abs(L1_kernel10_rz-L2_kernel10_rz);
     abs(L1_kernel12_rx-L2_kernel12_rx), abs(L1_kernel12_ry-L2_kernel12_ry), abs(L1_kernel12_rz-L2_kernel12_rz);
     abs(L1_kernel14_rx-L2_kernel14_rx), abs(L1_kernel14_ry-L2_kernel14_ry), abs(L1_kernel14_rz-L2_kernel14_rz);
     abs(L1_kernel16_rx-L2_kernel16_rx), abs(L1_kernel16_ry-L2_kernel16_ry), abs(L1_kernel16_rz-L2_kernel16_rz);
     abs(L1_kernel18_rx-L2_kernel18_rx), abs(L1_kernel18_ry-L2_kernel18_ry), abs(L1_kernel18_rz-L2_kernel18_rz)];
figure
rot_chart = bar(x_axis(2:end), kernels_r(2:end,:));
rx = [];
for i = 1:3
    rx = [rx ; rot_chart(i).XEndPoints];
end
hold on
errorbar(rx', kernels_r(2:end,:), err_r(2:end,:))
set(gca, 'YGrid', 'on', 'XGrid', 'off')
xlabel('kernal size [pixel]')
ylabel('standard deviation')
legend('x component','y component','z component')
title('kernel size vs. rotational stability')