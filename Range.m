classdef Range < matlab.mixin.Copyable
    properties (SetAccess = private)
        umin_ (1,1) double = 0;
        umax_ (1,1) double = 1;
        du_ (1,1) double = 1; % always umax_ - umin_
    end
    methods
        function SetRange(obj, umin, umax) % umin < umax
            arguments
                obj
                umin {mustBeNumeric, mustBeFinite}
                umax {mustBeNumeric, mustBeFinite, mustBeGreaterThan(umax, umin)}
            end
            obj.umin_ = umin;
            obj.umax_ = umax;
            obj.du_ = umax-umin;
        end
        
        function RestrictRange(obj, umin, umax) % umin < umax and within current interval
            arguments
                obj
                umin {mustBeNumeric, mustBeFinite}
                umax {mustBeNumeric, mustBeFinite, mustBeGreaterThan(umax, umin)}
            end
            assert(umin >= obj.umin_, 'Range.restrictRange: umin out of range: %g < %g',umin, obj.umin_);
            assert(umax <= obj.umax_, 'Range.restrictRange: umax out of range: %g > %g',umax, obj.umax_);
            obj.SetRange(umin, umax);
        end
        
        function rv = LinInterpol(obj, u, rhs1, rhs2) 
            % u must be scalar or row vector, rhsx scalar or column vector
            rv = ((obj.umax_ - u) .* rhs1 + (u - obj.umin_) .* rhs2) / obj.du_;
        end
        
        function CheckBounds(obj, u) % error if out of bounds
            arguments
                obj
                u (1,:) double
            end
            test = (u < obj.umin_) | (u > obj.umax_);
            if any(test)
                error('Range: u (%g) out of bounds ([%g;%g])',u, obj.umin_,obj.umax_);
            end
        end
    end
end
