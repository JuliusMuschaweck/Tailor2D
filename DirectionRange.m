classdef DirectionRange < Range
    methods (Abstract)
        k = Dir(obj, u)
        dk = dkdu(obj, u)
    end
end