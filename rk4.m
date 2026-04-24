% Normal RK4, except flag tells RK4 whether or not we need the previous
% force to be passed through.
function [t,x] = rk4(f,t,x0)
    x = zeros(length(t),length(x0));
    x(1,:) = x0;
    for i = 1:length(t)-1
        dt = t(i+1)-t(i);
        k1 = f(t(i),x(i,:)');
        k2 = f(t(i)+dt/2,x(i,:)'+dt/2*k1);
        k3 = f(t(i)+dt/2,x(i,:)'+dt/2*k2);
        k4 = f(t(i)+dt,x(i,:)'+dt*k3);
        x(i+1,:) = x(i,:)+(dt/6*(k1+2*k2+2*k3+k4))';

        % This statement will explicitely compute the
        % force input, so it can be passed, rather than some nonsensical
        % integrated value. This is important, because we need u_prev to
        % compute our input force with an INDI controller.
        k_ex = f(t(i+1),x(i+1,:)');
        x(i+1,end) = k_ex(end);
    end
end