function DDSMC_three_leaf()
%In 3D space, track a closed trefoil trajectory using a GVF-based DDSMC strategy and compare the desired and actual velocities.
%Figure 2(c) and Figure 3(a)
    clc; clear;

    %% ========== 1. Parameters, control gains ========== 
  m = 0.65; 
    g = 9.81;
    K_xi = diag([0.01, 0.01, 0.01]);
    J = diag([7.5e-3, 7.5e-3, 1.3e-2]);
    K_eta = diag([0.1, 0.1, 0.1]);
    mu1 = diag([0.3, 0.3, 0.3]);
    mu2 = diag([0.8, 0.8, 0.4]);
    lambda1 = diag([0.05,0.05,0.05]);
    lambda2 = diag([0.01,0.01,0.01]);
    k1 = diag([0.7,0.7,0.7]);
    k2 = diag([0.1,0.1,0.1]);
    epsilon = diag([0.01,0.01,0.01]);
    C1 = diag([0.05,0.05,0.05]);
    C2 = diag([0.05,0.06,0.05]);
    P1_hat = zeros(3,1);
    P2_hat = zeros(3,1);
    gamma1 = 0.005;
    gamma2 = 0.1;
    lambda2_diag = diag(lambda2);
    epsilon_diag = diag(epsilon);

    %% ========== 2. Simulation time ========== 
    t_final = 20;
    n_steps = 2000;
    t       = linspace(0, t_final, n_steps);
    dt      = t(2) - t(1);

    %% ========== 3. Initial states ========== 
    x = -2;   y = -3;   z = -4;
    x_dot = 0; y_dot = 0; z_dot = 0;
    phi = 0; theta = 0; psi = 0;
    phi_dot = 0; theta_dot = 0; psi_dot = 0;

    x_hist       = zeros(1,n_steps);
    y_hist       = zeros(1,n_steps);
    z_hist       = zeros(1,n_steps);
    error_hist   = zeros(1,n_steps);
    v_d_hist     = zeros(3,n_steps);
    v_actual_hist= zeros(3,n_steps);
    eta_d_hist = zeros(3,n_steps); 
    psi_des_last = 0;

    %% ========== 4. "Draw the desired trajectory (three_leaf) and mark the initial position." ========== 
    figure('Name','GVF + ISMC: 3-Lobed','NumberTitle','off');
    hold on; grid on; axis equal;
    thetas = linspace(0,2*pi,400);
    r0 = 2.0; a = 1.0;
    r_vals = r0 + a*cos(3*thetas);
    x_lobe = r_vals.*cos(thetas);
    y_lobe = r_vals.*sin(thetas);
    z_lobe = zeros(size(thetas));
    plot3(x_lobe, y_lobe, z_lobe, 'r--','LineWidth',2);
    plot3(x, y, z, 'bo','MarkerSize',6,'MarkerFaceColor','b');
    xlabel('x'); ylabel('y'); zlabel('z');
    title('3D Three‑lobe Curve Tracking');
    xlim([-5,5]); ylim([-5,5]); zlim([-5,5]);
    view(45,25);

    %% ========== 5. GVF gain ========== 
    k1_gvf = 3.8; 
    k2_gvf = 3.8;

    %% ========== 6. Main loop ========== 
    for i = 1:n_steps

        % (A) Define phi1, phi2 and the gradient
                % (A) phi1 and 2
        phi1 = z;
        phi2 = phi2_3lobe(x,y);
        n2   = [0;0;1];
        [dp2x,dp2y] = phi2_3lobe_grad(x,y);
        n1   = [dp2x; dp2y; 0];

        % (B) gvf
        cross_term = cross(n1,n2);
        v_d = cross_term ...
              - k1_gvf * phi2 * n1 ...
              - k2_gvf * phi1 * n2;

        % (C) outer
        v_actual = [x_dot; y_dot; z_dot];
        e_v      = v_d - v_actual;
        e_dot_v  = (v_d - v_actual)/dt;
        s1       = e_dot_v + mu1*e_v;
        P1_hat   = P1_hat + dt * gamma1 * (e_dot_v + mu1*e_v);

        a_cmd = C1*e_dot_v + K_xi*mu1*e_v + k1*s1 ...
                + (lambda1+epsilon).*tanh(s1) + [0;0;m*g] + P1_hat;

        % (D) inner
        
        if norm(v_d(1:2)) > 1e-6
        psi_raw = atan2(v_d(2), v_d(1));
        else
        psi_raw = psi; 
        end
        dpsi = psi_raw - psi_des_last;
        if dpsi > pi
        psi_raw = psi_raw - 2*pi;
        elseif dpsi < -pi
        psi_raw = psi_raw + 2*pi;
        end
        alpha = 0.1; 
        psi_des = (1 - alpha)*psi_des_last + alpha*psi_raw;
        psi_des_last = psi_des;
       
        norm_a = norm(a_cmd); if norm_a < 1e-6, norm_a = 1e-6; end
        phi_d = asin((a_cmd(1)*sin(psi_des) - a_cmd(2)*cos(psi_des))/norm_a);
        theta_d = atan((a_cmd(1)*cos(psi_des) + a_cmd(2)*sin(psi_des))/a_cmd(3));
        
        eta_d = [phi_d; theta_d; psi_des];
        if i == 1
        eta_d_dot = [0; 0; 0];
        else
        eta_d_dot = (eta_d - eta_d_hist(:, i-1)) / dt;
        end
        eta_d_hist(:, i) = eta_d;
       
        e_eta = eta_d - [phi; theta; psi];
        e_dot_eta = eta_d_dot - [phi_dot; theta_dot; psi_dot];
        s2 = e_dot_eta + mu2*e_eta;
        P2_hat = P2_hat + dt * gamma2 * s2;
        tau = C2*e_dot_eta + K_eta*mu2*e_eta + k2*s2 ...
        + (lambda2_diag + epsilon_diag) .* tanh(s2) ...
        + P2_hat;


      
        x_dot = x_dot + dt*(a_cmd(1)/m);
        y_dot = y_dot + dt*(a_cmd(2)/m);
        z_dot = z_dot + dt*((a_cmd(3)-m*g)/m);
        x     = x + dt*x_dot;
        y     = y + dt*y_dot;
        z     = z + dt*z_dot;

        phi_dot   = phi_dot   + dt*(tau(1)/J(1,1));
        theta_dot = theta_dot + dt*(tau(2)/J(2,2));
        psi_dot   = psi_dot   + dt*(tau(3)/J(3,3));
        phi   = phi   + dt*phi_dot;
        theta = theta + dt*theta_dot;
        psi   = psi   + dt*psi_dot;

        x_hist(i) = x;  y_hist(i) = y;  z_hist(i) = z;
        error_hist(i)   = abs(phi2);
        v_d_hist(:,i)      = v_d;
        v_actual_hist(:,i) = v_actual;

        % Dynamic drawing
        if mod(i,10)==0 || i==1
            plot3(x_hist(1:i), y_hist(1:i), z_hist(1:i), 'b','LineWidth',1.5);
            pause(0.01);
        end
        aaa = [phi,theta,psi];
        aaa
        
    end

    % Final trajectory
    plot3(x_hist, y_hist, z_hist, 'b','LineWidth',1.5);
    legend('Desired','Initial','Trajectory','Location','best');

    %% ========== 7. Tracking error curve ==========
    figure('Name','Tracking Error','NumberTitle','off');
    plot(t, error_hist, 'r','LineWidth',1.5);
    grid on; xlabel('Time (s)'); ylabel('|φ₂|');
    title('Tracking Error of Three‑lobe Shape');

%% ========== 8.Comparison of expected and actual speeds in different directions ==========
figure('Name','Desired vs Actual Velocity by Axis','NumberTitle','off');

% X
subplot(3,1,1);
plot(t, v_d_hist(1,:), 'k--', 'LineWidth',2); hold on;
plot(t, v_actual_hist(1,:), 'r-', 'LineWidth',1.5);
grid on;
ylabel('v_x (m/s)');
legend('v_{d,x}','v_x','Location','best');
title('X‑axis Velocity');

% Y
subplot(3,1,2);
plot(t, v_d_hist(2,:), 'k--', 'LineWidth',2); hold on;
plot(t, v_actual_hist(2,:), 'm-', 'LineWidth',1.5);
grid on;
ylabel('v_y (m/s)');
legend('v_{d,y}','v_y','Location','best');
title('Y‑axis Velocity');

% Z
subplot(3,1,3);
plot(t, v_d_hist(3,:), 'k--', 'LineWidth',2); hold on;
plot(t, v_actual_hist(3,:), 'c-', 'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('v_z (m/s)');
legend('v_{d,z}','v_z','Location','best');
title('Z‑axis Velocity');


end

%% ========== Implicit functions & gradients: phi2_3lobe ========== 
function val = phi2_3lobe(x, y)
    r0 = 2.0; a = 1.0;
    r  = sqrt(x^2 + y^2);
    theta = atan2(y, x);
    val = r - (r0 + a*cos(3*theta));
end

function [dfdx, dfdy] = phi2_3lobe_grad(x, y)
    r = sqrt(x^2 + y^2);
    if r < 1e-9
        dfdx = 0; dfdy = 0; return;
    end
    theta  = atan2(y, x);
    r0 = 2.0; a = 1.0;
    drdx = x/r; drdy = y/r;
    dthdx= -y/(x^2+y^2); dthdy= x/(x^2+y^2);
    dphi2dx = drdx - a*(-3*sin(3*theta))*dthdx;
    dphi2dy = drdy - a*(-3*sin(3*theta))*dthdy;
    dfdx = dphi2dx; dfdy = dphi2dy;
end
