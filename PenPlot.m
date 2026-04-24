% Basic plotting function. Plots pendulum angle and cart position. Also
% shows an animation of the cart moving.
function p = PenPlot(T,X,fq,fig_num)
    dx = 0.5;
    dy = 0.25;

    sq = 2+2*square(pi*T/7);

    figure(fig_num);clf;
    subplot(3,2,1);
    plot(T,X(:,1))
    hold on;
    plot(T,sq,LineStyle="--",Color="k");
    ylabel("Cart Displacement (m)");

    subplot(3,2,2);
    plot(T,X(:,2))
    hold on;
    yline(0,LineStyle="--",Color="k");
    ylabel("Cart Velocity (m/s)");

    subplot(3,2,3);
    plot(T,X(:,3))
    hold on;
    yline(0,LineStyle="--",Color="k");
    xlabel("Time (s)");
    ylabel("Pendulum Angle (rad)");

    subplot(3,2,4);
    plot(T,X(:,4))
    hold on;
    yline(0,LineStyle="--",Color="k");
    xlabel("Time (s)");
    ylabel("Pendulum Velocity (rad/s)");

    Cart_L2_Error = sqrt(cumtrapz(T, (sq - X(:,1)).^2));
    fprintf("Cart L2 Cumulative Error: %.4f\n", Cart_L2_Error(end));
    Pendulum_L2_Error = sqrt(cumtrapz(T, (0 - X(:,3)).^2));
    fprintf("Pendulum L2 Cumulative Error: %.4f\n", Pendulum_L2_Error(end));
    
    rx = X(:,1)-dx;
    ry = -dy;
    
    cx = X(:,1)+2*sin(X(:,3));
    cy = 2*cos(X(:,3));
    
    subplot(3,2,[5 6]);
    a = plot([X(1,1) cx(1)],[0 cy(1)]);
    hold on;
    r = rectangle('Position',[rx(1) ry(1) 2*dx 2*dy]);
    c = scatter(cx(1),cy(1), 4e2);
    
    allx = [cx;rx];
    ally = [cy;ry];
    lx = [min(allx)-0.5-dx max(allx)+0.5+dx];
    ly = [min(ally)-0.5 max(ally)+0.5];
    
    axis equal;
    title(strcat("t=",num2str(T(1))))
    xlabel("x (m)");
    ylabel("y (m)");
    
    % xlim([X(1,1)-3 X(1,1)+3])
    xlim(lx)
    ylim(ly)
    
    for i = 1:length(T)
        a.XData = [X(i,1) cx(i)];
        a.YData = [0 cy(i)];

        if isinf(rx(i)) || isnan(rx(i))
            break
        end

        r.Position = [rx(i) ry(1) 2*dx 2*dy];

        c.XData = cx(i);
        c.YData = cy(i);

        % xlim([X(i,1)-3 X(i,1)+3]);
        title(strcat("t=",num2str(T(i))))

        pause(1/fq)
    end
end