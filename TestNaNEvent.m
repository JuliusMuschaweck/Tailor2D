%%
% NaN doesnt work. But extending the range of the function and using an out of bounds flag with an event
% function does work
res = ode45(@odefun, [0,1], 1, odeset('Events',@eventsFcn));

%% 

test_ODE = RayTrace2D_ODEfun();
sol = test_ODE.Solve(@odefun_with_ok, [0 1], 1, odeset('RelTol',1e-6));

%%

function [dydt, ok] = odefun_with_ok(~, y) % simply y = exp(t)
    if y > 2 
        dydt = 2;
        ok = false;
    else
        dydt = y;
        ok = true;
    end
end


function dydt = odefun(~, y) % simply y = exp(t)
    if y > 2 
        dydt = 2;
    else
        dydt = y;
    end
end

function [value,isterminal,direction] = eventsFcn(~,y)
    if y > 2
        % value = 2 - y;
        value = -1;
    else
        % value = 20 - 10 * y;
        value = 1;
    end
    isterminal = 1;
    direction = 0;
end