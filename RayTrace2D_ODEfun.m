classdef RayTrace2D_ODEfun < handle
    properties (SetAccess = protected)
        odefun_with_ok_ 
    end
    methods
        function sol = Solve( obj, odefun_with_ok, tspan, y0, options)
            obj.odefun_with_ok_ = odefun_with_ok;
            myOdeFun = @(tt,yy) obj.i_odefun(tt,yy);
            myOpts = odeset(options, 'Events',@(ttt,yyy) obj.eventsFcn(ttt,yyy));
            sol = ode45(myOdeFun, tspan, y0, myOpts);
        end
    end
    methods (Access = private)
        function dydt = i_odefun(obj, t, y)
            dydt = obj.odefun_with_ok_(t, y);
        end
        function [value,isterminal,direction] = eventsFcn(obj, t, y)
            [~, ok] = obj.odefun_with_ok_(t, y);
            if ok
                value = 1;
            else
                value = -1;
            end
            isterminal = 1;
            direction = 0;
        end
    end
end