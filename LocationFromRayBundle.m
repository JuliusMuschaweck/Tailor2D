classdef LocationFromRayBundle < LocationRange
    properties (SetAccess = private)
        rb_ (1,1) RayBundle
    end
    
    methods 
        function obj = LocationFromRayBundle(rb)
            arguments
                rb (1,1) RayBundle
            end
            obj.rb_ = rb;
        end
        function p = Loc(obj, u)
            p = obj.rb_.Loc(u);
        end
        function d = Normal(obj, u)
            d = obj.rb_.Normal(u);
        end
        function t = Tangent(obj, u)
            t = obj.rb_.Tangent(u);
        end
    end    
end