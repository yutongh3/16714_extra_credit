clc;clear;
close all;

global robot;
robot = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);

goal = [0.4, 0, 0.6];
x0 = homeConfiguration(robot);
x0 = [x0'; zeros(7,1)];
endEffector = "EndEffector_Link";
taskInit = getTransform(robot, x0(1:7)', endEffector);
taskFinal = trvec2tform(goal) * axang2tform([0 1 0 pi]);
ik = inverseKinematics("RigidBodyTree", robot);
xT = ik(endEffector, taskFinal, [1 1 1 1 1 1], x0(1:7)');
xT = mod(xT, 2*pi);
xT = [xT'; zeros(7,1)];

Tmax = 5;
dt = 0.05;

nx = 14; % 7 pos + 7 vel
nu = 7;
A = 0.8*[eye(7), dt*eye(7); zeros(7), eye(7)];
B = 0.8*[zeros(7), dt*eye(7)]';

A_ref = [eye(7), dt*eye(7); zeros(7), eye(7)];
B_ref = [zeros(7), dt*eye(7)]';
% Cost weights
Q = blkdiag(1*eye(7), 1*eye(7)); % State cost weights
R = eye(nu);  % Correct dimension for control input

% Solve Discrete Algebraic Riccati Equation for LQR
[K_star, ~] = dlqr(A_ref, B_ref, Q, R);
A_star = A_ref - B_ref*K_star;

u = @(x, k) -k * (x - xT);
f = @(x, u) A * x + B * u;
f_ref = @(x, u) A_ref * x + B_ref * u;

% Simulate Trajectory with Kalman Filter
tlist = 0:dt:Tmax;
Nsteps = length(tlist)-1;
xlist = zeros(size(x0,1), Nsteps+1);
xlist(:,1) = x0; 
x = x0;
x_ref = x0;
x_ref_list = zeros(size(x0,1), Nsteps+1);
x_ref_list(:,1) = x_ref;

C = [A, B];
F = 0.01 * eye(nx+nu);
%%
% Nsteps = 31;
ulist = zeros(size(u(x0, K_star),1), Nsteps);
elist = zeros(nx, Nsteps+1);
Clist = zeros(nx, nx+nu, Nsteps+1);
Clist(:,:,1) = C;
% Rollout
for k = 1:Nsteps
    % find A and B using current C
    A_est = C(:, 1:nx);
    B_est = C(:, nx+1:end);

    % find control gain
    K = (B_est'*B_est)\B_est'*(A_est-A_star);

    u_curr = u(x_ref, K);

    % Tracking error
    e = x_ref - x;
    phi = [x; u_curr];
    F = pinv(pinv(F) + phi * phi');

    update_term = e * (phi' * F);
    C = C + update_term;

    x = A_est*x_ref + B_est*u_curr;

    x_ref = A_ref*x_ref + B_ref*u_curr;

    % Store trajectories
    xlist(:,k+1) = x;
    x_ref_list(:,k+1) = x_ref;
    ulist(:,k+1) = u_curr;
    elist(:,k+1) = e;
    a_matrix = C(:, 1:nx);
    a = mean(diag(a_matrix));
    b_matrix = C(:,nu+1:end);
    b = mean(diag(b_matrix));
    A_est = [a*eye(nu),b*eye(nu);zeros(nu),a*eye(nu)];
    B_est = [zeros(7), b*eye(7)]';
    C = [A_est, B_est];
    Clist(:,:,k+1) = C;
    % print k iter and a,b,c values
    fprintf('Iteration: %d, a: %f, b: %f\n', k, a, b);
end
%%
% 
% visualize_arm = 0;
% figure(1); 
% clist = zeros(length(tform2trvec(taskInit)),length(tlist)); 
% clist(:,1) = tform2trvec(taskInit)';
% for k = 2:Nsteps
%     x = x_ref_list(1:7,k)';
%     if visualize_arm
%         pause(dt);
%         show(robot,x,'PreservePlot',true,'Frames','off');
%         axis([-0.7 0.7 -0.7 0.7 -0.1 1.5]);
%     end
%     clist(:,k) = tform2trvec(getTransform(robot,x,endEffector))';
% end
% show(robot,x,'PreservePlot',true,'Frames','off');
% axis([-0.7 0.7 -0.7 0.7 -0.1 1.5]);
% hold on;
% plot3(clist(1,:),clist(2,:),clist(3,:),'k','LineWidth',2);
% hold on;
