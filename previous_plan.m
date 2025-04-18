clear

%%
MaxSpeed = 1000;

CMS_FOR_10_REVOLUTIONS = 60;
CM_PER_REVOLUTION = CMS_FOR_10_REVOLUTIONS / 10;
STEPS_PER_CM = 800 / CM_PER_REVOLUTION;

%%
reverseKinematics = @(x,y,z) [x + y; x - y; x - y + z];

%%
init_point_cm = [0; 0; 0];
final_point_cm = [1; 2; 3];

%
init_step = init_point_cm * STEPS_PER_CM;
final_step = final_point_cm * STEPS_PER_CM;

%
motor_init = reverseKinematics(init_step(1), init_step(2), init_step(3));
motor_final = reverseKinematics(final_step(1), final_step(2), final_step(3));

%
delta_motor = motor_final - motor_init;

%
abs_x = abs(delta_motor(1));
abs_y = abs(delta_motor(2));
abs_z = abs(delta_motor(3));

%
max_distance = max([abs_x, abs_y, abs_z]);

%
x_ratio = abs_x / max_distance;
y_ratio = abs_y / max_distance;
z_ratio = abs_z / max_distance;

%
x_speed = x_ratio * MaxSpeed * sign(delta_motor(1));
y_speed = y_ratio * MaxSpeed * sign(delta_motor(2));
z_speed = z_ratio * MaxSpeed * sign(delta_motor(3));


%
x_time = abs_x / abs(x_speed);
y_time = abs_y / abs(y_speed);
z_time = abs_z / abs(z_speed);

%
T_total = max([x_time, y_time, z_time]);

%%
disp(['X Motor：', num2str(x_speed)]);
disp(['Y Motor：', num2str(y_speed)]);
disp(['Z Motor：', num2str(z_speed)]);
disp(['X time cost：', num2str(x_time)]);
disp(['Y time cost：', num2str(y_time)]);
disp(['Z time cost：', num2str(z_time)]);
disp(['time cost：', num2str(T_total)]);
