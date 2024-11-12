classdef pusit_single_target < handle
    % pusit is composed of N targets' information, this example is for a
    % single target
    properties
        U (1, 1) double  % the number of the uav that occupies this target (only one), if not, -1
        
        d (1, 1) double {mustBeNonnegative} % distance between the target and the uav that occupies this target, if not, Inf
        tlabel (1, 1) double {mustBeInteger, mustBeNonnegative} % freshness of the recorded state
    end

    methods
        function obj = pusit_single_target()
            obj.U = -1;
            obj.d = 0;
            obj.tlabel = 0;
        end
    end
end