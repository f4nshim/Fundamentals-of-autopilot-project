clc, clear, close all
model_parameter
addpath('splines');
%% longitudinal direction

A_lon = [0 1;0 0];
B_lon = [0;ModelParams.Cm1/ModelParams.m];


%% lateral direction
vx = 25;
Kr = ModelParams.Dr*ModelParams.Cr*ModelParams.Br;
Kf = ModelParams.Df*ModelParams.Cf*ModelParams.Bf;

A_lat = [0 1 0 0;
    0 -2*(Kr+Kf)/ModelParams.m/vx 2*(Kr+Kf)/ModelParams.m -2*(ModelParams.lf*Kf-ModelParams.lr*Kr)/ModelParams.m/vx;
    0 0 0 1;
    0 -2*(ModelParams.lf*Kf-ModelParams.lr*Kr)/ModelParams.Iz/vx 2*(ModelParams.lf*Kf-ModelParams.lr*Kr)/ModelParams.Iz -2*(ModelParams.lf^2*Kf+ModelParams.lr^2*Kr)/ModelParams.Iz/vx];

B_lat = [0; 2*Kf/ModelParams.m; 0; 2*Kf*ModelParams.lf/ModelParams.Iz];

% rank(ctrb(A_lat,B_lat))

% Q_ctrl = diag([4,8,1,8]);
Q_ctrl = diag([1,2,1,2]);
R_ctrl = 30.0;
[ctrl_K,P,E] = lqr(A_lat, B_lat, Q_ctrl, R_ctrl);

%% lqi
C_lat = [1,0,0,0];
sys_lat = ss(A_lat,B_lat,C_lat,[]);

Q_ctrl = diag([1,1,1,1,0.1]);
R_ctrl = 2;
[ctrl_K_lqi,P,E] = lqi(sys_lat, Q_ctrl, R_ctrl);

%% add trajectory
%读取路径文件
% traj=load('trajectory.txt');
load('traj_diy.mat');

% %% init path
% %% import an plot track
% % use normal ORCA Track
% load Tracks/track2.mat
% % shrink track by half of the car widht plus safety margin
% % TODO implement orientation depending shrinking in the MPC constraints
% safteyScaling = 1.5;
% [track,track2] = borderAdjustment(track2,ModelParams,safteyScaling);
% trackWidth = norm(track.inner(:,1)-track.outer(:,1));
% 
% % plot shrinked and not shrinked track 
% % figure(1);
% % plot(track.outer(1,:),track.outer(2,:),'r')
% % hold on
% % plot(track.inner(1,:),track.inner(2,:),'r')
% % plot(track2.outer(1,:),track2.outer(2,:),'k')
% % plot(track2.inner(1,:),track2.inner(2,:),'k')
% 
% %% Fit spline to track
% % TODO spline function only works with regular spaced points.
% % Fix add function which given any center line and bound generates equlally
% % space tracks.
% traj.id = 1;
% [traj, borders] =splinify(track);
% tl = traj.ppy.breaks(end);
% traj.ppx = rmfield(traj.ppx, 'form');
% traj.ppy = rmfield(traj.ppy, 'form');
% traj.dppx = rmfield(traj.dppx, 'form');
% traj.dppy = rmfield(traj.dppy, 'form');
% traj.ddppx = rmfield(traj.ddppx, 'form');
% traj.ddppy = rmfield(traj.ddppy, 'form');
% 
% % store all data in one struct
% % global path_info
% path = struct('traj',traj,'borders',borders,'track_center',track.center,'tl',tl);
% % save path_info.mat path_info
% 
% traj_info = Simulink.Bus.createObject(traj);
% traj_bus = evalin('base', traj_info.busName);


%% 新建路径
% traj_x=traj(1,:);
% traj_y=traj(2,:);
% traj_k=traj(4,:);
% traj_xin=traj(5,:);
% traj_yin=traj(6,:);
% traj_xout=traj(7,:);
% traj_yout=traj(8,:);
% [~,pos]=sort(traj_k(:));
% x_in=cat(1,traj_xin(pos(1:4))*0.6+traj_x(pos(1:4))*0.4,pos(1:4)');
% y_in=cat(1,traj_yin(pos(1:4))*0.6+traj_y(pos(1:4))*0.4,pos(1:4)');
% x_out=cat(1,traj_xout(pos(end-3:end-1))*0.6+traj_x(pos(end-3:end-1))*0.4,pos(end-3:end-1)');
% y_out=cat(1,traj_yout(pos(end-3:end-1))*0.6+traj_y(pos(end-3:end-1))*0.4,pos(end-3:end-1)');
% x_point=sortrows(cat(2,x_in,x_out)',2);
% y_point=sortrows(cat(2,y_in,y_out)',2);
% i=1;
% while i<21
%     if x_point(i,2)~=0&&x_point(i,2)>30
%         if traj_k(x_point(i,2))<0
%             x_front=[traj_x(x_point(i,2)-30)*0.8+traj_xin(x_point(i,2)-30)*0.2 0];
%             x_after=[traj_x(x_point(i,2)+20)*0.4+traj_xin(x_point(i,2)+20)*0.6 0];
%             y_front=[traj_y(y_point(i,2)-30)*0.8+traj_yin(y_point(i,2)-30)*0.2 0];
%             y_after=[traj_y(y_point(i,2)+20)*0.4+traj_yin(y_point(i,2)+20)*0.6 0];
%         else
%             x_front=[traj_x(x_point(i,2)-40)*0.6+traj_xout(x_point(i,2)-40)*0.4 0];
%             x_after=[traj_x(x_point(i,2)+40)*0.2+traj_xout(x_point(i,2)+40)*0.8 0];
%             y_front=[traj_y(y_point(i,2)-40)*0.6+traj_yout(y_point(i,2)-40)*0.4 0];
%             y_after=[traj_y(y_point(i,2)+40)*0.2+traj_yout(y_point(i,2)+40)*0.8 0];
%         end
%         x_point=cat(1,x_point(1:i-1,:),x_front,x_point(i,:),x_after,x_point(i+1:end,:));
%         y_point=cat(1,y_point(1:i-1,:),y_front,y_point(i,:),y_after,y_point(i+1:end,:));
%         i=i+2;
%     else
%         i=i+1;
%     end    
% end
% 
% new_traj_x=cat(2,traj_x(1),x_point(:,1)',traj_x(end));
% new_traj_y=cat(2,traj_y(1),y_point(:,1)',traj_y(end));
% values = spcrv([[new_traj_x(1) new_traj_x new_traj_x(end)];[new_traj_y(1) new_traj_y new_traj_y(end)]],3,995);
% values(:,1)=[];
% values(:,end)=[];
% curv=[];
% yaw=[];
% for i=1:1:length(values)-2
%     func=polyfit(values(1,i:i+2),values(2,i:i+2),2);
%     syms x
%     y=x*x*func(1)+x*func(2)+func(3);
%     dy = simplify(diff(y, 'x'));
%     ddy = simplify(diff(y, 'x', 2));
%     c=simplify(ddy/(1+dy^2)^(3/2));
%     x=values(1,i+1);
%     c=eval(c);
%     curv=[curv c];
%     yaw=[yaw atan2(values(2,i+1)-values(2,i),values(1,i+1)-values(1,i))];
% end
% curv=[curv curv(end) curv(end)];
% yaw=[yaw yaw(end) yaw(end)];
% my_traj(1,:)=values(1,:);
% my_traj(2,:)=values(2,:);
% my_traj(3,:)=yaw;
% my_traj(4,:)=curv;
% 
% figure(1)
% plot(my_traj(1,:),my_traj(2,:))
% hold on
% plot(traj_x,traj_y)
% figure(2)
% plot(my_traj(3,:))
% hold on
% plot(traj(3,:))
% figure(3)
% plot(my_traj(4,:))
% hold on
% plot(traj_k)
% save my_traj.mat my_traj
load('my_traj.mat')



