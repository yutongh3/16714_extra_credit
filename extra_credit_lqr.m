% 16-714 Advanced Control for Robotics
% extra credit
% Yutong Huang
clc;clear;close all;

%% init
global robot;
robot = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);

goal = [0.4,0,0.6];
x0 = homeConfiguration(robot);
x0 = [x0;zeros(1,7)];
endEffector = "EndEffector_Link";
taskInit = getTransform(robot,x0(1,:),endEffector);
taskFinal = trvec2tform(goal)*axang2tform([0 1 0 pi]);
ik = inverseKinematics("RigidBodyTree",robot);
xT = ik(endEffector,taskFinal,[1 1 1 1 1 1],x0(1,:));
xT = mod(xT,2*pi);
xT = [xT;zeros(1,7)];


%% Joint space control (lqr)
Tmax = 5;
dt = 0.05; 
A = [1 dt; 0 1];
B = [0; 1];
Q = [1 0; 0 1];
R = 1;
figure(1); clf; axis([-0.7 0.7 -0.7 0.7 -0.1 1.5]);
show(robot,x0(1,:),'PreservePlot',true,'Frames','off');
axis([-0.7 0.7 -0.7 0.7 -0.1 1.5]);
[K,~] = dlqr(A,B,Q,R);
u = @(x) -K*(x-xT);
f = @(x,u) A*x + B*u;

% Simulate Trajectory
tlist = 0:dt:Tmax;
xlist = zeros(size(x0,1),size(x0,2),length(tlist)); 
xlist(:,:,1) = x0; 
ulist = zeros(size(u(x0),2),length(tlist)-1);
for k = 1:length(ulist)
    ulist(:,k) = u(xlist(:,:,k))';
    xlist(:,:,k+1) = f(xlist(:,:,k),ulist(:,k)');
end 

clist = zeros(length(tform2trvec(taskInit)),length(tlist)); 
clist(:,1) = tform2trvec(taskInit)';

%% visualize
for k = 2:length(tlist)
    x = xlist(1,:,k);
    pause(dt);
    show(robot,x,'PreservePlot',true,'Frames','off');
    axis([-0.7 0.7 -0.7 0.7 -0.1 1.5]);
    clist(:,k) = tform2trvec(getTransform(robot,x,endEffector))';
end
hold on;
plot3(clist(1,:),clist(2,:),clist(3,:),'k','LineWidth',2);
axis([-0.7 0.7 -0.7 0.7 -0.1 1.5]);

%% plot control tarj
figure(2);
for i = 1:7
    plot(2:length(tlist), ulist(i,:)); hold on;
end

