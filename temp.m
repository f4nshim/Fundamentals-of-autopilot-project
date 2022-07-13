function output=temp()

traj_x=traj(1,:);      %给定x坐标
traj_y=traj(2,:);      %给定y坐标
traj_k=traj(4,:);      %曲率

%% 新建路径
traj_xin=traj(5,:);
traj_yin=traj(6,:);
traj_xout=traj(7,:);
traj_yout=traj(8,:);
[As,pos]=sort(traj_k(:));
x_in=cat(1,traj_xin(pos(1:4))*0.7+traj_x(pos(1:4))*0.3,pos(1:4)');
y_in=cat(1,traj_yin(pos(1:4))*0.7+traj_y(pos(1:4))*0.3,pos(1:4)');
x_out=cat(1,traj_xout(pos(end-3:end-1))*0.7+traj_x(pos(end-3:end-1))*0.3,pos(end-3:end-1)');
y_out=cat(1,traj_yout(pos(end-3:end-1))*0.7+traj_y(pos(end-3:end-1))*0.3,pos(end-3:end-1)');
x_point=sortrows(cat(2,x_in,x_out)',2);
y_point=sortrows(cat(2,y_in,y_out)',2);
i=1;
while i<21
    if x_point(i,2)~=0&&x_point(i,2)>30
        x_front=[traj_x(x_point(i,2)-50) 0];
        x_after=[traj_x(x_point(i,2)+50) 0];
        y_front=[traj_y(y_point(i,2)-50) 0];
        y_after=[traj_y(y_point(i,2)+50) 0];
        x_point=[x_point(1:i-1,:);x_front;x_point(i,:);x_after;x_point(i+1:end,:)];
        y_point=[y_point(1:i-1,:);y_front;y_point(i,:);y_after;y_point(i+1:end,:)];
        i=i+2;
    else
        i=i+1;
    end
end
new_traj_x=cat(2,traj_x(1),x_point(:,1)',traj_x(end));
new_traj_y=cat(2,traj_y(1),y_point(:,1)',traj_y(end));
values = spcrv([[new_traj_x(1) new_traj_x new_traj_x(end)];[new_traj_y(1) new_traj_y new_traj_y(end)]],3,700);
figure(1)
plot(new_traj_x,new_traj_y,'c*')
hold on
plot(values(1,:),values(2,:))
hold off
values(:,1)=[];
values(:,end)=[];
curv=[];
yaw=[];
for i=1:1:length(values)-2
    func=polyfit(values(1,i:i+2),values(2,i:i+2),2);
    syms x
    y=x*x*func(1)+x*func(2)+func(3);
    dy = simplify(diff(y, 'x'));
    ddy = simplify(diff(y, 'x', 2));
    c=ddy/(1+dy^2)^(3/2);
    x=values(1,i+1);
    c=eval(c);
    curv=[curv c];
    yaw=[yaw atan2(values(2,i+1)-values(2,i),values(1,i+1)-values(1,i))];
end
curv=[curv(1) curv curv(end)];
yaw=[yaw(1) yaw yaw(end)];
output=[values(1,:);values(2,:);curv,yaw];


