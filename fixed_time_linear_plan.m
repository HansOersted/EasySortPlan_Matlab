function fixed_time_linear_plan()
    % parameters
    MaxSpeed = 1000;
    dt = 0.001;

    CMS_FOR_10_REVOLUTIONS = 60;
    CM_PER_REVOLUTION = CMS_FOR_10_REVOLUTIONS / 10;
    STEPS_PER_CM = 800 / CM_PER_REVOLUTION;

    A = STEPS_PER_CM * [1 1 0;
                        1 -1 0;
                        1 -1 1];

    init_point = [2; 0; 7];
    final_point = [1; 5; 3];

    delta_position = final_point - init_point;
    delta_motor = A * delta_position;
    T_candidate = abs(delta_motor) / MaxSpeed;
    T_min = max(T_candidate);
    position_vel = delta_position / T_min;
    motor_speed = A * position_vel;

    t_vec = 0:dt:T_min;
    num_steps = length(t_vec);

    traj_pos = init_point + position_vel * t_vec;
    traj_motor = (A * traj_pos)';

    motor_velocity_history = repmat(motor_speed', num_steps - 1, 1);

    traj_velocity_history = repmat(position_vel', num_steps - 1, 1);

    x = traj_pos(1, :);
    y = traj_pos(2, :);
    z = traj_pos(3, :);
    motorX = traj_motor(:,1);
    motorY = traj_motor(:,2);
    motorZ = traj_motor(:,3);

    % Plot 1: Trajectory Position
    figure;
    plot(t_vec, x, 'r', 'LineWidth', 1.5); hold on;
    plot(t_vec, y, 'g', 'LineWidth', 1.5);
    plot(t_vec, z, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Position (cm)');
    legend('x(t)', 'y(t)', 'z(t)');
    title('Trajectory Position (Fixed-Time Linear)');
    grid on;

    % Plot 2: Motor Position
    figure;
    plot(t_vec, motorX, 'r', 'LineWidth', 1.5); hold on;
    plot(t_vec, motorY, 'g', 'LineWidth', 1.5);
    plot(t_vec, motorZ, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Motor Position (steps)');
    legend('motorX(t)', 'motorY(t)', 'motorZ(t)');
    title('Motor Position (Fixed-Time Linear)');
    grid on;

    % Plot 3: Trajectory Velocity
    figure;
    plot(t_vec(2:end), traj_velocity_history(:,1), 'r', 'LineWidth', 1.5); hold on;
    plot(t_vec(2:end), traj_velocity_history(:,2), 'g', 'LineWidth', 1.5);
    plot(t_vec(2:end), traj_velocity_history(:,3), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Velocity (cm/s)');
    legend('v_x(t)', 'v_y(t)', 'v_z(t)');
    title('Trajectory Velocity (Fixed-Time Linear)');
    grid on;

    % Plot 4: Motor Velocity
    figure;
    plot(t_vec(2:end), motor_velocity_history(:,1), 'r', 'LineWidth', 1.5); hold on;
    plot(t_vec(2:end), motor_velocity_history(:,2), 'g', 'LineWidth', 1.5);
    plot(t_vec(2:end), motor_velocity_history(:,3), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Motor Velocity (steps/s)');
    legend('v_{motorX}', 'v_{motorY}', 'v_{motorZ}');
    title('Motor Velocity (Fixed-Time Linear)');
    grid on;

    disp(['least time: ', num2str(T_min)]);
    disp('position_vel:'); disp(position_vel);
    disp('motor_speed:'); disp(motor_speed);
end
