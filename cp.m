classdef cp < matlab.mixin.Copyable
    properties
        a = 42
    end
    methods
        function SetA(obj, aa)
            obj.a = aa;
        end
    end
end