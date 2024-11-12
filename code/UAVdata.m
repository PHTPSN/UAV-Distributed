classdef UAVdata < handle
    properties
        n (1, 1) double {mustBeInteger, mustBeNonnegative}  % number of uavs
        targ_positions (2, :) double % the position of each target
        uav (1, :) single_uav
        v = 0.15;    % max velocity
        cr = 2;     % Communication radius
        sr = 0.5;     % Security radius

        start_pos (2, 1) double
        end_pos 
        obstacles obstacle

        target_scatter (1, 1) matlab.graphics.chart.primitive.Scatter
        uav_scatter (1, 1) matlab.graphics.chart.primitive.Scatter
        target_textlist (1, :) cell
        uav_textlist (1, :) cell
    end

    events
        Next_time
        Exchange_information
    end
    
    methods
        function obj = UAVdata(N, tpositions, uavpositions, start_position, end_position)
            arguments
                N (1, 1) double {mustBeInteger, mustBeNonnegative}
                tpositions (2, :) double
                uavpositions (2, :) double
                start_position
                end_position
            end

            obj.n = N;
            for i = 1:obj.n
                obj.targ_positions(:, i) = tpositions(:, i);
            end
            for i = 1:obj.n
                obj.uav(i) = single_uav(i, uavpositions(:, i), obj);
            end

            for i = 1:obj.n
                obj.uav(i) = obj.uav(i).preliminary_decision(1:obj.n, obj);
            end

            obj.start_pos = start_position;
            obj.end_pos = repmat(end_position', 1, N) + tpositions;
            obj.obstacles = obstacle((sqrt(N)-1)*5*rand(2, sqrt(N)) + sqrt(N) - 1,...
                1.5*rand(1, sqrt(N)), obj.end_pos);

            figure
            hold on
            for i = 1:sqrt(N)
                theta = linspace(0, 2*pi, 50);
                x = obj.obstacles.pos(1, i) + obj.obstacles.radius(i) * cos(theta);  
                y = obj.obstacles.pos(2, i) + obj.obstacles.radius(i) * sin(theta);
                fill(x, y, 'black');
            end
            



            % obj.target_scatter = scatter(tpositions(1, :), tpositions(2, :), 'g');
            obj.uav_scatter = scatter(uavpositions(1, :), uavpositions(2, :), 3, 'r', 'Marker','.');
            for i = 1:N
                % obj.target_textlist{i} = text(tpositions(1, i), tpositions(2, i), {i});
                obj.uav_textlist{i} = text(uavpositions(1, i), uavpositions(2, i), {i});
            end
            axis equal
            xlim([-1 6*sqrt(N)])
            ylim([-1 6*sqrt(N)])
        end
        
        function obj = run(obj)
            
            for t = 1:300
                notify(obj, "Exchange_information");
                notify(obj, "Next_time");
                
                for j = 1:obj.n
                    [dx, dy] = obj.obstacles.calculate_movement_direction_and_step(obj.targ_positions(:,j)', (sqrt(obj.n)-1)/2, obj.v*0.8, j);
                    
                    obj.targ_positions(:,j) = obj.targ_positions(:,j) + [dx, dy]';
                end

                % delete(obj.target_scatter)
                % obj.target_scatter = scatter(obj.targ_positions(1, :), obj.targ_positions(2, :), 'g');
                % for i = 1:obj.n
                %     set(obj.target_textlist{i}, 'Position', [obj.targ_positions(:, i)' 0]);
                % end

                delete(obj.uav_scatter)
                uavpositions = zeros(2, obj.n);
                for i = 1:obj.n
                    uavpositions(:, i) = obj.uav(i).position;
                end
                obj.uav_scatter = scatter(uavpositions(1, :), uavpositions(2, :), 3, 'r', 'Marker','.');
                for i = 1:obj.n
                    set(obj.uav_textlist{i}, 'Position', [uavpositions(:, i)' 0]);
                end
                pause(0.02)
            end

            obj.targ_positions = obj.end_pos;
            for t = 1:100
                notify(obj, "Exchange_information");
                notify(obj, "Next_time");
                

                % delete(obj.target_scatter)
                % obj.target_scatter = scatter(obj.targ_positions(1, :), obj.targ_positions(2, :), 'g');
                % for i = 1:obj.n
                %     set(obj.target_textlist{i}, 'Position', [obj.targ_positions(:, i)' 0]);
                % end

                delete(obj.uav_scatter)
                uavpositions = zeros(2, obj.n);
                for i = 1:obj.n
                    uavpositions(:, i) = obj.uav(i).position;
                end
                obj.uav_scatter = scatter(uavpositions(1, :), uavpositions(2, :), 3, 'r', 'Marker','.');
                for i = 1:obj.n
                    set(obj.uav_textlist{i}, 'Position', [uavpositions(:, i)' 0]);
                end
                pause(0.02)
            end

        end

    end
end