classdef single_uav < handle
    properties
        number (1, 1) double {mustBeInteger, mustBeNonnegative} % the number of itself
        target (1, 1) double {mustBeInteger, mustBeNonnegative} % current target number
        position (2, 1) double
        d2targets (1, :) double {mustBeNonnegative}  % the distance between each target and the uav
        d2otheruavs (1, :) double {mustBeNonnegative}  % the distance to other uavs, to itself is 0
        adjacentset (1, :) double {mustBeInteger, mustBeNonnegative}  % uav numbers in communication radius
        pusit (1, :) pusit_single_target
    end

    methods
        % Attention: callback function doesn't need "obj = "
        function obj = single_uav(num, pos, uavdata)
            arguments (Input)
                num (1, 1) double {mustBeInteger, mustBeNonnegative}
                pos (2, 1) double
                uavdata UAVdata
            end
            obj.number = num;
            obj.position = pos;
            obj.d2targets = [];
            obj.d2otheruavs = [];
            obj.adjacentset = [];
            for i = 1:uavdata.n
                obj.pusit(i) = pusit_single_target;
            end

            addlistener(uavdata, "Next_time", @(src, evnt)obj.step_forward(src, evnt));
            addlistener(uavdata, "Exchange_information", @(src, evnt)obj.change_state(src, evnt));
        end

        
        function step_forward(obj, uavdata, ~)
            arguments (Input)
                obj single_uav
                uavdata UAVdata
                ~
            end

            current_distance = obj.d2targets(obj.target);
            if current_distance > uavdata.sr
                k = 1;
            else
                k = current_distance / uavdata.sr;
            end
            obj.position = obj.position + k * uavdata.v / current_distance * (uavdata.targ_positions(:, obj.target) - obj.position); 
            obj.d2targets(obj.target) = norm(uavdata.targ_positions(:, obj.target) - obj.position);
            obj.pusit(obj.target).d = obj.d2targets(obj.target);
            obj.pusit(obj.target).tlabel = 0;
            % move

            for j = 1:uavdata.n
                obj.pusit(j).tlabel = obj.pusit(j).tlabel + 1;
            end
        end

        function change_state(obj, uavdata, ~)
            arguments (Input)
                obj single_uav
                uavdata UAVdata
                ~
            end
            
            obj = obj.search_adjacentset(uavdata);
            if ~isempty(obj.adjacentset)
                obj = update_pusit(obj, uavdata);
                obj = obj.secondary_decision(uavdata);
            end
        end

        function obj = search_adjacentset(obj, uavdata)
            arguments (Input)
                obj single_uav
                uavdata UAVdata
            end
            obj.adjacentset = [];
            for i = 1:uavdata.n
                obj.d2otheruavs(i) = norm(uavdata.uav(i).position - obj.position);
                if obj.d2otheruavs(i) < uavdata.cr && i ~= obj.number
                    obj.adjacentset = [obj.adjacentset i];
                end
            end
        end

        function obj = preliminary_decision(obj, unoccupied_targets, uavdata)
            for i = 1:length(unoccupied_targets)
                obj.d2targets(unoccupied_targets(i)) = norm(uavdata.targ_positions(:, unoccupied_targets(i)) - obj.position);
            end
            [~, index] = min(obj.d2targets(unoccupied_targets));
            obj.target = unoccupied_targets(index);
            obj.pusit(obj.target).U = obj.number;
            obj.pusit(obj.target).d = obj.d2targets(obj.target);
            obj.pusit(obj.target).tlabel = 0;
        end

        function obj = secondary_decision(obj, uavdata)
            unoccupied_targets = [];
            if obj.pusit(obj.target).U ~= obj.number
                for j = 1:uavdata.n
                    if obj.pusit(j).U == -1
                        unoccupied_targets = [unoccupied_targets j];
                    end
                end
                if ~isempty(unoccupied_targets)
                    obj = preliminary_decision(obj, unoccupied_targets, uavdata);
                end
            end
        end
        
        function obj = update_pusit(obj, uavdata)
            % 排查重复
            % Ulist = zeros(1, 16);
            % dlist = zeros(1, 16);
            % for j = 1:uavdata.n
            %     Ulist(j) = obj.pusit(j).U;
            %     dlist(j) = obj.pusit(j).d;
            % end
            % for i = 1:uavdata.n
            %     targetlisti = find(Ulist == i);
            %     if ~isempty(targetlisti)
            %         [~, index] = min(dlist(targetlisti));
            %         for j = 1:length(targetlisti)
            %             if j ~= index
            %                 obj.pusit(targetlisti(j)).U = -1;
            %                 obj.pusit(targetlisti(j)).d = 0;
            %             end
            %         end
            %     end
            % end

            for j = 1:uavdata.n
                own_target_uav = obj.pusit(j).U;
                all_U_same = true;
                dlist = []; % pusit(j).d of all uavs
                tlabellist = [];    % pusit(j).tlabel of all uavs
                for i = 1:length(obj.adjacentset)
                    ii = obj.adjacentset(i);
                    if all_U_same == true && uavdata.uav(ii).pusit(j).U ~= own_target_uav
                        all_U_same = false;
                    end
                    dlist = [dlist uavdata.uav(ii).pusit(j).d];
                    tlabellist = [tlabellist uavdata.uav(ii).pusit(j).tlabel];
                end
                if all_U_same
                    [~, newest] = min(tlabellist);
                    if tlabellist(newest) < obj.pusit(j).tlabel % don't forget itself
                        obj.pusit(j) = uavdata.uav(obj.adjacentset(newest)).pusit(j);
                    end
                else
                    dlist = dlist - uavdata.v * tlabellist; % 由于临近障碍物/目标点会减速，这项还欠考虑
                    [~, farthest] = max(dlist);
                    if dlist(farthest) > obj.pusit(j).d - uavdata.v * obj.pusit(j).tlabel
                        obj.pusit(j) = uavdata.uav(obj.adjacentset(farthest)).pusit(j);
                    end
                end
            end
        end

    end
end