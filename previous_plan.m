function previous_plan()
    % parameters
    MaxSpeed = 1000;
    dt = 0.001;

    CMS_FOR_10_REVOLUTIONS = 60;
    CM_PER_REVOLUTION = CMS_FOR_10_REVOLUTIONS / 10;
    STEPS_PER_CM = 800 / CM_PER_REVOLUTION;

    reverseKinematics = @(x,y,z) [x + y; x - y; x - y + z] .* STEPS_PER_CM;
    forwardKinematics = @(motorX, motorY, motorZ) [
        (motorX + motorY)/2 / STEPS_PER_CM;
        (motorX - motorY)/2 / STEPS_PER_CM;
        (motorZ - motorY) / STEPS_PER_CM
    ];

    init_point_cm = [2; 0; 7];
    goal_point_cm = [1; 5; 3];

    current_cm = init_point_cm;
    current_motor = reverseKinematics(current_cm(1), current_cm(2), current_cm(3));
    goal_motor = reverseKinematics(goal_point_cm(1), goal_point_cm(2), goal_point_cm(3));

    % time and history
    t = 0;
    traj_time = t;
    traj_x = current_cm(1);
    traj_y = current_cm(2);
    traj_z = current_cm(3);

    traj_motorX = current_motor(1);
    traj_motorY = current_motor(2);
    traj_motorZ = current_motor(3);

    motor_velocity_history = [];
    traj_velocity_history = [];

    % simulation loop
    while true
        delta_motor = goal_motor - current_motor;
        abs_motor = abs(delta_motor);
        max_distance = max(abs_motor);

        % exit condition
        if all(abs_motor < 1)
            break;
        end

        ratio = abs_motor / max_distance;
        speed = ratio * MaxSpeed .* sign(delta_motor);  % motor speed (steps/s)
        step = speed * dt;

        % update position
        current_motor = current_motor + step;
        current_cm = forwardKinematics(current_motor(1), current_motor(2), current_motor(3));

        % convert motor speed to trajectory velocity
        vx = (speed(1) + speed(2))/2 / STEPS_PER_CM;
        vy = (speed(1) - speed(2))/2 / STEPS_PER_CM;
        vz = (speed(3) - speed(2)) / STEPS_PER_CM;

        % update time and record
        t = t + dt;
        traj_time(end+1) = t;
        traj_x(end+1) = current_cm(1);
        traj_y(end+1) = current_cm(2);
        traj_z(end+1) = current_cm(3);

        traj_motorX(end+1) = current_motor(1);
        traj_motorY(end+1) = current_motor(2);
        traj_motorZ(end+1) = current_motor(3);

        motor_velocity_history(end+1, :) = speed;
        traj_velocity_history(end+1, :) = [vx, vy, vz];
    end

    % Plot 1: trajectory position
    figure;
    plot(traj_time, traj_x, 'r', 'LineWidth', 1.5); hold on;
    plot(traj_time, traj_y, 'g', 'LineWidth', 1.5);
    plot(traj_time, traj_z, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Position (cm)');
    legend('x(t)', 'y(t)', 'z(t)');
    title('Trajectory Position');
    grid on;

    % Plot 2: motor position
    figure;
    plot(traj_time, traj_motorX, 'r'); hold on;
    plot(traj_time, traj_motorY, 'g');
    plot(traj_time, traj_motorZ, 'b');
    xlabel('Time (s)');
    ylabel('Motor Position (steps)');
    legend('motorX(t)', 'motorY(t)', 'motorZ(t)');
    title('Motor Step Trajectory');
    grid on;

    % Plot 3: trajectory velocity
    figure;
    plot(traj_time(2:end), traj_velocity_history(:,1), 'r', 'LineWidth', 1.5); hold on;
    plot(traj_time(2:end), traj_velocity_history(:,2), 'g', 'LineWidth', 1.5);
    plot(traj_time(2:end), traj_velocity_history(:,3), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Trajectory Velocity (cm/s)');
    legend('v_x(t)', 'v_y(t)', 'v_z(t)');
    title('Trajectory Velocity History');
    grid on;

    % Plot 4: motor velocity
    figure;
    plot(traj_time(2:end), motor_velocity_history(:,1), 'r', 'LineWidth', 1.5); hold on;
    plot(traj_time(2:end), motor_velocity_history(:,2), 'g', 'LineWidth', 1.5);
    plot(traj_time(2:end), motor_velocity_history(:,3), 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Motor Velocity (steps/s)');
    legend('v_{motorX}(t)', 'v_{motorY}(t)', 'v_{motorZ}(t)');
    title('Motor Velocity History');
    grid on;

    % print total time
    fprintf('Total simulated time: %.3f s\n', t);
end
