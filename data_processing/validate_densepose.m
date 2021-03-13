%% validate DensePose
clc; clear; close all

% read data: horizontal index _ vertical index
h10_v0 = csvread('data/DP_validate10_0.csv');
h30_v0 = csvread('data/DP_validate30_0.csv');
h50_v0 = csvread('data/DP_validate50_0.csv');
h70_v0 = csvread('data/DP_validate70_0.csv');
h90_v0 = csvread('data/DP_validate90_0.csv');

h10_v1 = csvread('data/DP_validate10_1.csv');
h30_v1 = csvread('data/DP_validate30_1.csv');
h50_v1 = csvread('data/DP_validate50_1.csv');
h70_v1 = csvread('data/DP_validate70_1.csv');
h90_v1 = csvread('data/DP_validate90_1.csv');

h10_v_1 = csvread('data/DP_validate10_-1.csv');
h30_v_1 = csvread('data/DP_validate30_-1.csv');
h50_v_1 = csvread('data/DP_validate50_-1.csv');
h70_v_1 = csvread('data/DP_validate70_-1.csv');
h90_v_1 = csvread('data/DP_validate90_-1.csv');

reg1 = [h10_v_1(:,1),h30_v_1(:,1),h50_v_1(:,1),h70_v_1(:,1),h90_v_1(:,1), ...
        h10_v0(:,1),h30_v0(:,1),h50_v0(:,1),h70_v0(:,1),h90_v0(:,1), ...
        h10_v1(:,1),h30_v1(:,1),h50_v1(:,1),h70_v1(:,1),h90_v1(:,1)];
    
reg2 = [h10_v_1(:,2),h30_v_1(:,2),h50_v_1(:,2),h70_v_1(:,2),h90_v_1(:,2), ...
        h10_v0(:,2),h30_v0(:,2),h50_v0(:,2),h70_v0(:,2),h90_v0(:,2), ...
        h10_v1(:,2),h30_v1(:,2),h50_v1(:,2),h70_v1(:,2),h90_v1(:,2)];    

reg3 = [h10_v_1(:,3),h30_v_1(:,3),h50_v_1(:,3),h70_v_1(:,3),h90_v_1(:,3), ...
        h10_v0(:,3),h30_v0(:,3),h50_v0(:,3),h70_v0(:,3),h90_v0(:,3), ...
        h10_v1(:,3),h30_v1(:,3),h50_v1(:,3),h70_v1(:,3),h90_v1(:,3)];    

reg4 = [h10_v_1(:,4),h30_v_1(:,4),h50_v_1(:,4),h70_v_1(:,4),h90_v_1(:,4), ...
        h10_v0(:,4),h30_v0(:,4),h50_v0(:,4),h70_v0(:,4),h90_v0(:,4), ...
        h10_v1(:,4),h30_v1(:,4),h50_v1(:,4),h70_v1(:,4),h90_v1(:,4)];
    
reg1_map = reshape(mean(reg1),5,3)';    
reg2_map = reshape(mean(reg2),5,3)';    
reg3_map = reshape(mean(reg3),5,3)';    
reg4_map = reshape(mean(reg4),5,3)';    

figure
% -------------------------------------------------------------------------
subplot(2,2,1)
bar3(reg1_map, 0.4);
hold on
line([1,1],[1,1],[reg1_map(1,1),reg1_map(1,1)+std(reg1(:,1))],'color','r','LineWidth',1.5)
line([2,2],[1,1],[reg1_map(1,2),reg1_map(1,2)+std(reg1(:,2))],'color','r','LineWidth',1.5)
line([3,3],[1,1],[reg1_map(1,3),reg1_map(1,3)+std(reg1(:,3))],'color','r','LineWidth',1.5)
line([4,4],[1,1],[reg1_map(1,4),reg1_map(1,4)+std(reg1(:,4))],'color','r','LineWidth',1.5)
line([5,5],[1,1],[reg1_map(1,5),reg1_map(1,5)+std(reg1(:,5))],'color','r','LineWidth',1.5)

line([1,1],[2,2],[reg1_map(2,1),reg1_map(2,1)+std(reg1(:,6))],'color','r','LineWidth',1.5)
line([2,2],[2,2],[reg1_map(2,2),reg1_map(2,2)+std(reg1(:,7))],'color','r','LineWidth',1.5)
line([3,3],[2,2],[reg1_map(2,3),reg1_map(2,3)+std(reg1(:,8))],'color','r','LineWidth',1.5)
line([4,4],[2,2],[reg1_map(2,4),reg1_map(2,4)+std(reg1(:,9))],'color','r','LineWidth',1.5)
line([5,5],[2,2],[reg1_map(2,5),reg1_map(2,5)+std(reg1(:,10))],'color','r','LineWidth',1.5)

line([1,1],[3,3],[reg1_map(3,1),reg1_map(3,1)+std(reg1(:,11))],'color','r','LineWidth',1.5)
line([2,2],[3,3],[reg1_map(3,2),reg1_map(3,2)+std(reg1(:,12))],'color','r','LineWidth',1.5)
line([3,3],[3,3],[reg1_map(3,3),reg1_map(3,3)+std(reg1(:,13))],'color','r','LineWidth',1.5)
line([4,4],[3,3],[reg1_map(3,4),reg1_map(3,4)+std(reg1(:,14))],'color','r','LineWidth',1.5)
line([5,5],[3,3],[reg1_map(3,5),reg1_map(3,5)+std(reg1(:,15))],'color','r','LineWidth',1.5)

title('target1')
xlabel({'HORIZ','offset', '[mm]'})
xticklabels({'-40','-20','0','20','40'})
ylabel({'VERT','offset','[mm]'})
yticklabels({'40','0','-40'})
zlabel('error [pixel]')
zlim([0,50])

% -------------------------------------------------------------------------
subplot(2,2,2)
bar3(reg2_map, 0.4);
hold on
line([1,1],[1,1],[reg2_map(1,1),reg2_map(1,1)+std(reg2(:,1))],'color','r','LineWidth',1.5)
line([2,2],[1,1],[reg2_map(1,2),reg2_map(1,2)+std(reg2(:,2))],'color','r','LineWidth',1.5)
line([3,3],[1,1],[reg2_map(1,3),reg2_map(1,3)+std(reg2(:,3))],'color','r','LineWidth',1.5)
line([4,4],[1,1],[reg2_map(1,4),reg2_map(1,4)+std(reg2(:,4))],'color','r','LineWidth',1.5)
line([5,5],[1,1],[reg2_map(1,5),reg2_map(1,5)+std(reg2(:,5))],'color','r','LineWidth',1.5)

line([1,1],[2,2],[reg2_map(2,1),reg2_map(2,1)+std(reg2(:,6))],'color','r','LineWidth',1.5)
line([2,2],[2,2],[reg2_map(2,2),reg2_map(2,2)+std(reg2(:,7))],'color','r','LineWidth',1.5)
line([3,3],[2,2],[reg2_map(2,3),reg2_map(2,3)+std(reg2(:,8))],'color','r','LineWidth',1.5)
line([4,4],[2,2],[reg2_map(2,4),reg2_map(2,4)+std(reg2(:,9))],'color','r','LineWidth',1.5)
line([5,5],[2,2],[reg2_map(2,5),reg2_map(2,5)+std(reg2(:,10))],'color','r','LineWidth',1.5)

line([1,1],[3,3],[reg2_map(3,1),reg2_map(3,1)+std(reg2(:,11))],'color','r','LineWidth',1.5)
line([2,2],[3,3],[reg2_map(3,2),reg2_map(3,2)+std(reg2(:,12))],'color','r','LineWidth',1.5)
line([3,3],[3,3],[reg2_map(3,3),reg2_map(3,3)+std(reg2(:,13))],'color','r','LineWidth',1.5)
line([4,4],[3,3],[reg2_map(3,4),reg2_map(3,4)+std(reg2(:,14))],'color','r','LineWidth',1.5)
line([5,5],[3,3],[reg2_map(3,5),reg2_map(3,5)+std(reg2(:,15))],'color','r','LineWidth',1.5)

title('target2')
xlabel({'HORIZ','offset', '[mm]'})
xticklabels({'-40','-20','0','20','40'})
ylabel({'VERT','offset','[mm]'})
yticklabels({'40','0','-40'})
zlabel('error [pixel]')
zlim([0,50])

% -------------------------------------------------------------------------
subplot(2,2,3)
bar3(reg3_map, 0.4);
hold on
line([1,1],[1,1],[reg3_map(1,1),reg3_map(1,1)+std(reg3(:,1))],'color','r','LineWidth',1.5)
line([2,2],[1,1],[reg3_map(1,2),reg3_map(1,2)+std(reg3(:,2))],'color','r','LineWidth',1.5)
line([3,3],[1,1],[reg3_map(1,3),reg3_map(1,3)+std(reg3(:,3))],'color','r','LineWidth',1.5)
line([4,4],[1,1],[reg3_map(1,4),reg3_map(1,4)+std(reg3(:,4))],'color','r','LineWidth',1.5)
line([5,5],[1,1],[reg3_map(1,5),reg3_map(1,5)+std(reg3(:,5))],'color','r','LineWidth',1.5)

line([1,1],[2,2],[reg3_map(2,1),reg3_map(2,1)+std(reg3(:,6))],'color','r','LineWidth',1.5)
line([2,2],[2,2],[reg3_map(2,2),reg3_map(2,2)+std(reg3(:,7))],'color','r','LineWidth',1.5)
line([3,3],[2,2],[reg3_map(2,3),reg3_map(2,3)+std(reg3(:,8))],'color','r','LineWidth',1.5)
line([4,4],[2,2],[reg3_map(2,4),reg3_map(2,4)+std(reg3(:,9))],'color','r','LineWidth',1.5)
line([5,5],[2,2],[reg3_map(2,5),reg3_map(2,5)+std(reg3(:,10))],'color','r','LineWidth',1.5)

line([1,1],[3,3],[reg3_map(3,1),reg3_map(3,1)+std(reg3(:,11))],'color','r','LineWidth',1.5)
line([2,2],[3,3],[reg3_map(3,2),reg3_map(3,2)+std(reg3(:,12))],'color','r','LineWidth',1.5)
line([3,3],[3,3],[reg3_map(3,3),reg3_map(3,3)+std(reg3(:,13))],'color','r','LineWidth',1.5)
line([4,4],[3,3],[reg3_map(3,4),reg3_map(3,4)+std(reg3(:,14))],'color','r','LineWidth',1.5)
line([5,5],[3,3],[reg3_map(3,5),reg3_map(3,5)+std(reg3(:,15))],'color','r','LineWidth',1.5)

title('target3')
xlabel({'HORIZ','offset', '[mm]'})
xticklabels({'-40','-20','0','20','40'})
ylabel({'VERT','offset','[mm]'})
ylabel({'VERT','offset', '[mm]'})
zlabel('error [pixel]')
zlim([0,50])

% -------------------------------------------------------------------------
subplot(2,2,4)
bar3(reg4_map, 0.4);
hold on
line([1,1],[1,1],[reg4_map(1,1),reg4_map(1,1)+std(reg4(:,1))],'color','r','LineWidth',1.5)
line([2,2],[1,1],[reg4_map(1,2),reg4_map(1,2)+std(reg4(:,2))],'color','r','LineWidth',1.5)
line([3,3],[1,1],[reg4_map(1,3),reg4_map(1,3)+std(reg4(:,3))],'color','r','LineWidth',1.5)
line([4,4],[1,1],[reg4_map(1,4),reg4_map(1,4)+std(reg4(:,4))],'color','r','LineWidth',1.5)
line([5,5],[1,1],[reg4_map(1,5),reg4_map(1,5)+std(reg4(:,5))],'color','r','LineWidth',1.5)

line([1,1],[2,2],[reg4_map(2,1),reg4_map(2,1)+std(reg4(:,6))],'color','r','LineWidth',1.5)
line([2,2],[2,2],[reg4_map(2,2),reg4_map(2,2)+std(reg4(:,7))],'color','r','LineWidth',1.5)
line([3,3],[2,2],[reg4_map(2,3),reg4_map(2,3)+std(reg4(:,8))],'color','r','LineWidth',1.5)
line([4,4],[2,2],[reg4_map(2,4),reg4_map(2,4)+std(reg4(:,9))],'color','r','LineWidth',1.5)
line([5,5],[2,2],[reg4_map(2,5),reg4_map(2,5)+std(reg4(:,10))],'color','r','LineWidth',1.5)

line([1,1],[3,3],[reg4_map(3,1),reg4_map(3,1)+std(reg4(:,11))],'color','r','LineWidth',1.5)
line([2,2],[3,3],[reg4_map(3,2),reg4_map(3,2)+std(reg4(:,12))],'color','r','LineWidth',1.5)
line([3,3],[3,3],[reg4_map(3,3),reg4_map(3,3)+std(reg4(:,13))],'color','r','LineWidth',1.5)
line([4,4],[3,3],[reg4_map(3,4),reg4_map(3,4)+std(reg4(:,14))],'color','r','LineWidth',1.5)
line([5,5],[3,3],[reg4_map(3,5),reg4_map(3,5)+std(reg4(:,15))],'color','r','LineWidth',1.5)

title('target4')
xlabel({'HORIZ','offset', '[mm]'})
xticklabels({'-40','-20','0','20','40'})
ylabel({'VERT','offset','[mm]'})
yticklabels({'40','0','-40'})
zlabel('error [pixel]')
zlim([0,50])

set(gcf, 'Position',  [500, 200, 550, 500])