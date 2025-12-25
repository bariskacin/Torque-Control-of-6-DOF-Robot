clc; clear all;

run("Robot_model.m")
robot = robot_model;

dt = 0.01;

orientation = roty(15);

%       X       Y     Z
p1 = [ 0.2;  -0.51; 0.25];   
p2 = p1;   
p3 = [ 0.25;  -0.39; 0.35];   
p4 = [ 0.3;  -0.45; 0.13];   
p5 = [ 0.2;  -0.57; 0.03];   
p6 = p1;   
p7 = [ 2.0;   2.0;  2.0];

T_cart = cell(1,7);
P = {p1,p2,p3,p4,p5,p6,p7};
for k = 1:7
    T_cart{k} = [orientation, P{k}; 0 0 0 1];
end

Tf_cart = [1 1 1 1 1 1];  

Tf_joint = 1;

init_pos = deg2rad([0 0 0 0 0 0]');

ik = inverseKinematics("RigidBodyTree", robot);
weights = [0.25 0.25 0.25 1 1 1];
initialGuess = zeros(6,1);

[q_T1, ~] = ik('body6', T_cart{1}, weights, initialGuess);

t_joint = 0:dt:Tf_joint;
[q, qd, qdd, ~] = cubicpolytraj([init_pos q_T1], [0 Tf_joint], t_joint);

q_traj   = q;
qd_traj  = qd;
qdd_traj = qdd;

q_traj(:,end)   = [];
qd_traj(:,end)  = [];
qdd_traj(:,end) = [];

for s = 1:6
    t_seg = 0:dt:Tf_cart(s);

    [tforms, vel, ~] = transformtraj(T_cart{s}, T_cart{s+1}, [0 Tf_cart(s)], t_seg);

    Ns = numel(t_seg);
    q_seg  = zeros(6, Ns);
    qd_seg = zeros(6, Ns);

    initialGuess = q_traj(:,end); 
    for i = 1:Ns
        [q_seg(:,i), ~] = ik('body6', tforms(:,:,i), weights, initialGuess);
        initialGuess = q_seg(:,i);

        J = geometricJacobian(robot, q_seg(:,i), 'body6');
        qd_seg(:,i) = lsqminnorm(J, vel(:,i)); 
    end

    qdd_seg = zeros(6, Ns);
    for i = 2:Ns
        qdd_seg(:,i) = (qd_seg(:,i) - qd_seg(:,i-1)) / dt;
    end

    if s < 6
        q_seg(:,end)   = [];
        qd_seg(:,end)  = [];
        qdd_seg(:,end) = [];
    end

    q_traj   = [q_traj  q_seg];
    qd_traj  = [qd_traj qd_seg];
    qdd_traj = [qdd_traj qdd_seg];
end

N = size(q_traj,2);
t = (0:N-1) * dt;  

q1_traj = q_traj(1,:); q2_traj = q_traj(2,:); q3_traj = q_traj(3,:);
q4_traj = q_traj(4,:); q5_traj = q_traj(5,:); q6_traj = q_traj(6,:);

assignin('base','q1', timetable(seconds(t'), q1_traj'));
assignin('base','q2', timetable(seconds(t'), q2_traj'));
assignin('base','q3', timetable(seconds(t'), q3_traj'));
assignin('base','q4', timetable(seconds(t'), q4_traj'));
assignin('base','q5', timetable(seconds(t'), q5_traj'));
assignin('base','q6', timetable(seconds(t'), q6_traj'));

q1d_traj = qd_traj(1,:); q2d_traj = qd_traj(2,:); q3d_traj = qd_traj(3,:);
q4d_traj = qd_traj(4,:); q5d_traj = qd_traj(5,:); q6d_traj = qd_traj(6,:);

assignin('base','q1d', timetable(seconds(t'), q1d_traj'));
assignin('base','q2d', timetable(seconds(t'), q2d_traj'));
assignin('base','q3d', timetable(seconds(t'), q3d_traj'));
assignin('base','q4d', timetable(seconds(t'), q4d_traj'));
assignin('base','q5d', timetable(seconds(t'), q5d_traj'));
assignin('base','q6d', timetable(seconds(t'), q6d_traj'));

q1dd_traj = qdd_traj(1,:); q2dd_traj = qdd_traj(2,:); q3dd_traj = qdd_traj(3,:);
q4dd_traj = qdd_traj(4,:); q5dd_traj = qdd_traj(5,:); q6dd_traj = qdd_traj(6,:);

assignin('base','q1dd', timetable(seconds(t'), q1dd_traj'));
assignin('base','q2dd', timetable(seconds(t'), q2dd_traj'));
assignin('base','q3dd', timetable(seconds(t'), q3dd_traj'));
assignin('base','q4dd', timetable(seconds(t'), q4dd_traj'));
assignin('base','q5dd', timetable(seconds(t'), q5dd_traj'));
assignin('base','q6dd', timetable(seconds(t'), q6dd_traj'));

Kp = 100*eye(6);
Kd = 20*eye(6);

sim("Final_proje_sim.slx", 6.16);