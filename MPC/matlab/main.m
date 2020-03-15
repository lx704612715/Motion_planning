%%main
clear all; close all;

no_step = 2000;
% use 20 points 
K = 20;
% start position
px_0 =0;
py_0 = 8;
pz_0 = 20;
%% all threshold should be positiv
%limit v
max_v_xy = 6;
min_v_xy = 6;
max_v_z = 6;
min_v_z = 1;
%limit_a
max_a_xy = 3;
min_a_xy = 3;
max_a_z = 3;
min_a_z = 1;
%limit_j
max_j_xy = 3;
min_j_xy = 3;
max_j_z = 2;
min_j_z = 2; 

%% main function
[Target_px, Target_py, Target_pz] = get_TargetTrajectory(no_step, K);
log_x = mpc_solver(px_0, max_v_xy, min_v_xy, max_a_xy, min_a_xy, max_j_xy, min_j_xy, Target_px, no_step);
log_y = mpc_solver(py_0, max_v_xy, min_v_xy, max_a_xy, min_a_xy, max_j_xy, min_j_xy, Target_py, no_step);
log_z = mpc_solver(pz_0, max_v_z, min_v_z, max_a_z, min_a_z, max_j_z, min_j_z, Target_pz, no_step);

%%plot
px = log_x(:, 2);
py = log_y(:, 2);
pz = log_z(:, 2);

vx = log_x(:, 3);
vy = log_y(:, 3);
vz = log_z(:, 3);

ax = log_x(:, 4);
ay = log_y(:, 4);
az = log_z(:, 4);

jx = log_x(:, 5);
jy = log_y(:, 5);
jz = log_z(:, 5);

t = log_x(:, 1);

figure(1)
p1 = plot3(px, py, pz, 'linewidth', 1);
hold on
p2 = plot3(Target_px, Target_py, Target_pz, '--', 'linewidth', 3);
hold off
title('Trajectory')
legend([p1, p2],{'real trajectory', 'reference trajectory'})

figure(2)
plot(t, px-Target_px(1:no_step)', t,py-Target_py(1:no_step)' , t,pz-Target_pz(1:no_step)');
title('Error of Position')
legend({'Error px','Error py', 'Error pz'})

figure(3)
plot(t, vx, t, vy , t, vz);
title('Velocity')
legend({'vx','vy', 'vz'})

figure(4)
plot(t, ax, t, ay, t, az);
title('Acceleration')
legend({'ax','ay', 'az'})

figure(5)
plot(t, jx, t, jy, t, jz);
title('Jerk')
legend({'jx','jy', 'jz'})
