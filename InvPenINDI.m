function xdot = InvPenINDI(t,x,K,fxc,gxc,M,m,l,noise)
    x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4); u_prev = x(5);
    g = 9.81;

    % Correct dynamics, for plant simulation
    Mr = 30;
    mr = 20;
    lr = 3.4;

    f_x = [x2;
           (1/((Mr/mr)+sin(x3)^2))*(x4^2*lr*sin(x3)-g*sin(x3)*cos(x3));
           x4;
           (1/(lr*(Mr/mr)+lr*sin(x3)^2))*(-x4^2*lr*cos(x3)*sin(x3)+(Mr+mr)*g*sin(x3)/mr)];
    g_x = [0;
           1/(Mr+mr*sin(x3)^2);
           0;
           -cos(x3)/(lr*Mr+lr*mr*sin(x3)^2)];
    %==========================================
    
    % Using the model to calculate the true state based of the previous
    % input, this in real implementation would need to be directly
    % measured, or computed via something like an EKF.
    xdot_prev = f_x + g_x*u_prev;

    % Noise in xdot
    xdot_meas = xdot_prev;
    if noise
        xdot_meas = awgn(xdot_meas, 5);
    end

    % Reference signals
    xr = [2+2*square(pi*t/7);
          0;
          0;
          0;];

    % Noise in x
    x_meas = x(1:4);
    if noise
        x_meas = awgn(x_meas, 5);
    end

    % Dynamics used for controller formulation
    x1 = x_meas(1); x2 = x_meas(2); x3 = x_meas(3); x4 = x_meas(4);
    f_x_c = fxc(x2,x3,x4,M,m,l,g); % Completely unused, because INDI
    g_x_c = gxc(x2,x3,x4,M,m,l,g);

    e = xr-x_meas;

    v = K*e;

    du = pinv(g_x_c)*(v-xdot_meas);
    u = u_prev+du;

    xdot = [f_x+g_x*u;u];
end