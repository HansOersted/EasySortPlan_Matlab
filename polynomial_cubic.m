function polynomial_cubic()
    %% Parameters
    MaxMotorSpeed = 1000;     % steps/s
    MaxMotorAccel = 1000;     % steps/s^2
    dt = 0.001;               % simulation time step (s)

    CMS_FOR_10_REVOLUTIONS = 60;
    CM_PER_REVOLUTION = CMS_FOR_10_REVOLUTIONS / 10;
    STEPS_PER_CM = 800 / CM_PER_REVOLUTION;

    %% Reverse kinematics matrix (motor = A * [x; y; z])
    A = STEPS_PER_CM * [1 1 0;
                        1 -1 0;
                        1 -1 1];

    %% Task space start/end
    p0 = [2; 0; 7];
    pf = [1; 5; 3];

    %% Binary search for minimum feasible time
    T_low = 0.1;
    T_high = 10;
    T_tol = 1e-4;

    while (T_high - T_low > T_tol)
        T_try = (T_low + T_high)/2;
        [ok] = check_motor_constraint(p0, pf, T_try, A, STEPS_PER_CM, dt, MaxMotorSpeed, MaxMotorAccel);
        if ok
            T_high = T_try;
        else
            T_low = T_try;
        end
    end
    T_min = T_high;

    %% Generate final trajectory
    [~, t_vec, p, v, a] = check_motor_constraint(p0, pf, T_min, A, STEPS_PER_CM, dt, MaxMotorSpeed, MaxMotorAccel);

    %% Compute task space path length
    v_norm = vecnorm(v);
    s_vec = cumtrapz(t_vec, v_norm);

    %% Plot 1: Task Position vs Time
    figure;
    plot(t_vec, p(1,:), 'r', t_vec, p(2,:), 'g', t_vec, p(3,:), 'b', 'LineWidth', 1.5);
    legend('x(t)', 'y(t)', 'z(t)');
    xlabel('Time (s)'); ylabel('Position (cm)'); grid on;
    title('Task Position vs Time');

    %% Plot 2: Motor Position vs Time
    motor_pos = A * p;
    figure;
    plot(t_vec, motor_pos(1,:), 'r', t_vec, motor_pos(2,:), 'g', t_vec, motor_pos(3,:), 'b', 'LineWidth', 1.5);
    legend('motorX', 'motorY', 'motorZ');
    xlabel('Time (s)'); ylabel('Steps'); grid on;
    title('Motor Position vs Time');

    %% Plot 3: Velocity Components vs Path Length (Separate Axes)
    figure;
    subplot(3,1,1);
    plot(s_vec, v(1,:), 'r', 'LineWidth', 1.5);
    ylabel('v_x (cm/s)'); grid on; title('v_x vs Path Length');

    subplot(3,1,2);
    plot(s_vec, v(2,:), 'g', 'LineWidth', 1.5);
    ylabel('v_y (cm/s)'); grid on; title('v_y vs Path Length');

    subplot(3,1,3);
    plot(s_vec, v(3,:), 'b', 'LineWidth', 1.5);
    xlabel('Path Length (cm)'); ylabel('v_z (cm/s)'); grid on; title('v_z vs Path Length');

    %% Plot 4: Motor Velocity/Acceleration vs Time
    motor_v = A * v;
    motor_a = A * a;
    figure;
    subplot(2,1,1);
    plot(t_vec, motor_v(1,:), 'r', t_vec, motor_v(2,:), 'g', t_vec, motor_v(3,:), 'b', 'LineWidth', 1.2);
    legend('v_{motorX}', 'v_{motorY}', 'v_{motorZ}');
    ylabel('Velocity (steps/s)'); grid on;
    title('Motor Velocity vs Time');

    subplot(2,1,2);
    plot(t_vec, motor_a(1,:), 'r--', t_vec, motor_a(2,:), 'g--', t_vec, motor_a(3,:), 'b--', 'LineWidth', 1.2);
    legend('a_{motorX}', 'a_{motorY}', 'a_{motorZ}');
    xlabel('Time (s)'); ylabel('Accel (steps/s^2)'); grid on;
    title('Motor Acceleration vs Time');

    fprintf('Minimum feasible time T = %.4f s\n', T_min);
end

function [ok, t_vec, p, v, a] = check_motor_constraint(p0, pf, T, A, STEPS_PER_CM, dt, vmax, amax)
    t_vec = 0:dt:T;
    n = length(t_vec);

    % Coefficients for cubic polynomial p(t) = a0 + a1 t + a2 t^2 + a3 t^3
    a0 = p0;
    a1 = zeros(3,1);
    D = pf - p0;
    a2 = 3*D / T^2;
    a3 = -2*D / T^3;

    p = zeros(3,n);
    v = zeros(3,n);
    a = zeros(3,n);
    for i = 1:n
        t = t_vec(i);
        p(:,i) = a0 + a1*t + a2*t^2 + a3*t^3;
        v(:,i) = a1 + 2*a2*t + 3*a3*t^2;
        a(:,i) = 2*a2 + 6*a3*t;
    end

    motor_v = A * v;
    motor_a = A * a;

    if any(abs(motor_v(:)) > vmax + 1e-3) || any(abs(motor_a(:)) > amax + 1e-3)
        ok = false;
    else
        ok = true;
    end
end
