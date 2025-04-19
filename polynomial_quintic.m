function polynomial_quintic()
    %% Parameters
    MaxMotorSpeed = 1000;     % steps/s
    MaxMotorAccel = 1000;     % steps/s^2
    dt = 0.001;

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
        [ok] = check_quintic_constraint(p0, pf, T_try, A, dt, MaxMotorSpeed, MaxMotorAccel);
        if ok
            T_high = T_try;
        else
            T_low = T_try;
        end
    end
    T_min = T_high;

    %% Generate final trajectory
    [~, t_vec, p, v, a] = check_quintic_constraint(p0, pf, T_min, A, dt, MaxMotorSpeed, MaxMotorAccel);

    %% Compute path length
    v_norm = vecnorm(v);
    s_vec = cumtrapz(t_vec, v_norm);

    %% Plot task space position
    figure;
    plot(t_vec, p(1,:), 'r', t_vec, p(2,:), 'g', t_vec, p(3,:), 'b', 'LineWidth', 1.5);
    legend('x(t)', 'y(t)', 'z(t)');
    xlabel('Time (s)'); ylabel('Position (cm)'); grid on;
    title('Task Position vs Time');

    %% Plot motor space position
    motor_pos = A * p;
    figure;
    plot(t_vec, motor_pos(1,:), 'r', t_vec, motor_pos(2,:), 'g', t_vec, motor_pos(3,:), 'b', 'LineWidth', 1.5);
    legend('motorX', 'motorY', 'motorZ');
    xlabel('Time (s)'); ylabel('Steps'); grid on;
    title('Motor Position vs Time');

    %% Velocity components vs path length
    figure;
    subplot(3,1,1);
    plot(s_vec, v(1,:), 'r'); ylabel('v_x (cm/s)'); grid on; title('v_x vs s');
    subplot(3,1,2);
    plot(s_vec, v(2,:), 'g'); ylabel('v_y (cm/s)'); grid on; title('v_y vs s');
    subplot(3,1,3);
    plot(s_vec, v(3,:), 'b'); xlabel('s (cm)'); ylabel('v_z (cm/s)'); grid on; title('v_z vs s');

    %% Motor velocity/acceleration
    motor_v = A * v;
    motor_a = A * a;
    figure;
    subplot(2,1,1);
    plot(t_vec, motor_v(1,:), 'r', t_vec, motor_v(2,:), 'g', t_vec, motor_v(3,:), 'b');
    legend('v_{motorX}', 'v_{motorY}', 'v_{motorZ}'); ylabel('Velocity (steps/s)'); grid on;
    title('Motor Velocity');

    subplot(2,1,2);
    plot(t_vec, motor_a(1,:), 'r--', t_vec, motor_a(2,:), 'g--', t_vec, motor_a(3,:), 'b--');
    legend('a_{motorX}', 'a_{motorY}', 'a_{motorZ}'); xlabel('Time (s)'); ylabel('Accel (steps/s^2)'); grid on;
    title('Motor Acceleration');

    fprintf('Minimum feasible time T = %.4f s\n', T_min);
end

function [ok, t_vec, p, v, a] = check_quintic_constraint(p0, pf, T, A, dt, vmax, amax)
    t_vec = 0:dt:T;
    n = length(t_vec);

    % 5th-order polynomial: p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    a0 = p0;
    a1 = zeros(3,1); % v0 = 0
    a2 = zeros(3,1); % a0 = 0
    D = pf - p0;
    a3 = 10*D / T^3;
    a4 = -15*D / T^4;
    a5 = 6*D / T^5;

    p = zeros(3,n);
    v = zeros(3,n);
    a = zeros(3,n);
    for i = 1:n
        t = t_vec(i);
        p(:,i) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
        v(:,i) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
        a(:,i) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
    end

    motor_v = A * v;
    motor_a = A * a;
    if any(abs(motor_v(:)) > vmax + 1e-3) || any(abs(motor_a(:)) > amax + 1e-3)
        ok = false;
    else
        ok = true;
    end
end