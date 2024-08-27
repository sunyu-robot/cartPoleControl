%% Cartpole nmpc,lmpc,LQR
% dependencies : casadi
% model : cartpole (no friction)
import casadi.*
clear;clc;
format long
%------------------------------config-----------------------------------%
h = 0.04; % step
n = 8; % whole time
N = n/h;  % whole step
t = zeros(1,N); % time vec

% state 
dq = zeros(4,N);
q = zeros(4,N);
% input
u = zeros(1,N);
% initial state
q0 = [0, pi,0,0];
q(:,1) = q0;
% desired state
qd = [0,0,0,0];

% cart-pole dynamics
global mc mp l g
mc = 1; mp = 1; % cart mass, pole mass
l = 1; % pole length
g = 9.8; % gravity

% Linearization of the operating point for LMPC and LQR (> pi/4 will result in failure)
% LQR
% Q = diag([100,10,1, 1 ]); % x q dx dq
% R = 1; % fx
% K = cartPoleLQR(Q, R); % the LQR controller computes the gain only once.

% LMPC
% p_h = 80;
% Q = diag([100,10,1, 1 ]); % x q dx dq
% R = 1; % fx

% NMPC
p_h = 20;
Q = diag([600,1000,100, 40 ]); % x q dx dq
R = 1e-3; % fx
%% loop begin
for i = 1:N-1
%     u(i) = K*(qd'-q(:,i)); % LQR
%     u(i) = cartPoleLMPC(Q, R, q(:,i), qd, p_h, h);
    u(i) = cartPoleNMPC(Q, R, q(:,i), qd, p_h, h); 

    % dynamics (rk4)
    k1 = cartPoleDynamics(u(i), q(:, i));
    k2 = cartPoleDynamics(u(i), q(:, i) + 0.5 * h * k1);
    k3 = cartPoleDynamics(u(i), q(:, i) + 0.5 * h * k2);
    k4 = cartPoleDynamics(u(i), q(:, i) + h * k3);
    
    q(:, i+1) = q(:, i) + (h/6) * (k1 + 2 * k2 + 2 * k3 + k4);
    % update time
    t(i+1) = t(i)+h;
end
%% draw figure
% state fig
figure(1)
plot(t,q(1,:),'-','linewidth',2);hold on;
plot(t,q(2,:),'-','linewidth',2);hold on;title('joint state');grid on;
legend('x','\theta');
% input fig
figure(2)
plot(t,u,'-','linewidth',2);title('input');grid on;
legend('F');
% anime
% video
video = VideoWriter('cartpole.mp4', 'MPEG-4');
video.FrameRate = 1/h; 
open(video);

figure(3)
for i = 1:N
    % pole
    plot([q(1,i) q(1,i)+l*sin(q(2,i))],[0 l*cos(q(2,i))],'LineWidth',4);
    % cart
    width = 0.6;height = 0.3;
    rectangle('Position', [q(1,i)-width/2,-height/2, width, height], 'FaceColor', 'none', 'EdgeColor', 'k','LineWidth',2);
    hold on;
    % ball
    radius = 60;
    scatter(q(1,i)+l*sin(q(2,i)), l*cos(q(2,i)), radius, 'filled', 'r');
    hold off;
    xlim([-2 2]); ylim([-2 2]);  
    xlabel('X'); ylabel('Y');  
    grid on; 
    frame = getframe(gcf);
    writeVideo(video, frame);
end
close(video);