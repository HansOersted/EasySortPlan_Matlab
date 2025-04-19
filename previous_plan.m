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

    t = 0;
    traj_time = t;
    traj_x = current_cm(1);
    traj_y = current_cm(2);
    traj_z = current_cm(3);

    % simulation
    while true
        delta_motor = goal_motor - current_motor;
        abs_motor = abs(delta_motor);
        max_distance = max(abs_motor);

        % exit
        if all(abs_motor < 1)
            break;
        end

        ratio = abs_motor / max_distance;
        speed = ratio * MaxSpeed .* sign(delta_motor);
        step = speed * dt;

        current_motor = current_motor + step;

        current_cm = forwardKinematics(current_motor(1), current_motor(2), current_motor(3));

        t = t + dt;
        traj_time(end+1) = t;
        traj_x(end+1) = current_cm(1);
        traj_y(end+1) = current_cm(2);
        traj_z(end+1) = current_cm(3);
    end

    figure;
    plot(traj_time, traj_x, 'r', 'LineWidth', 1.5); hold on;
    plot(traj_time, traj_y, 'g', 'LineWidth', 1.5);
    plot(traj_time, traj_z, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Position (cm)');
    legend('x(t)', 'y(t)', 'z(t)');
    title('Dynamic Coordinated Motion to One Goal Point');
    grid on;

    fprintf('Total simulated time: %.3f s\n', t);
end
