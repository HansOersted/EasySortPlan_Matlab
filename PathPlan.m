clear

%%
MaxSpeed = 1000;

CMS_FOR_10_REVOLUTIONS = 60;
CM_PER_REVOLUTION = CMS_FOR_10_REVOLUTIONS / 10;
STEPS_PER_CM = 800 / CM_PER_REVOLUTION;

%%
% motorX = (x + y) * STEPS_PER_CM;
% motorY = (x - y) * STEPS_PER_CM;
% motorZ = (x - y + z) * STEPS_PER_CM;

% Constraint: d_moter/dt belonds to (-MaxSpeed, MaxSpeed)

A = STEPS_PER_CM * [1 1 0 ;
                    1 -1 0;
                    1 -1 1];

%%
init_point = [0;0;0];
final_point = [1,2,3];
%
delta_position = final_point' - init_point;
delta_motor = A * delta_position;
%
T_candidate = abs(delta_motor) / MaxSpeed;
T_min = max(T_candidate);
%
position_vel = delta_position / T_min;
motor_speed = A * position_vel;

%%
disp(['least time: ', num2str(T_min)]);
disp('position_vel:');
disp(position_vel);
disp('motor_speed:');
disp(motor_speed);