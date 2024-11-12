classdef obstacle < handle
    properties
        pos (2, :) double
        radius (1, :) double
        end_pos
    end

    methods
        function obj = obstacle(positions, rs, end_position)
            obj.pos = positions;
            obj.radius = rs;
            obj.end_pos = end_position;
        end



        function [dx, dy] = calculate_movement_direction_and_step(obj, current_pos, square_side, desired_speed, j)  
     
            dx = 0;  
            dy = 0;  
              
            
            center_position = current_pos + [square_side/2, square_side/2];  
              
            
            attractive_force = 1.5*attractive_potential_gradient(center_position, obj.end_pos(j)');
            repulsive_force = zeros(1, 2);  
            for i = 1:length(obj.radius)  
                obstacle_force = repulsive_potential_gradient(center_position, obj.pos(:, i)', obj.radius(i), square_side, square_side/2);
                repulsive_force = repulsive_force + obstacle_force;
            end  
              
            
           
            total_force = attractive_force - repulsive_force;
              
             
            if norm(total_force) > 0  
                direction = total_force / norm(total_force) ;
            else  
                
                return;  
            end  
              
            
            step_size = desired_speed / norm(direction);  
              
           
            dx = step_size * direction(1);  
            dy = step_size * direction(2);  
              
            
        end  
          
          
        
          
         
        

    end
end