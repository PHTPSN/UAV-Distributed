N = 36;
obstacles_pos = (sqrt(N)-1)*10*rand(2, 2*sqrt(N)) + sqrt(N) + 4;
obstacles_radius = 5*rand(1, 2*sqrt(N));

figure
hold on
for i = 1:2*sqrt(N)
    theta = linspace(0, 2*pi, 50);
    x = obstacles_pos(1, i) + obstacles_radius(i) * cos(theta);  
    y = obstacles_pos(2, i) + obstacles_radius(i) * sin(theta);
    fill(x, y, 'black');
end

xs = zeros(1, 100);
ys = zeros(1, 100);
for i = 2:100
    [dx, dy] = calculate_movement_direction_and_step([xs(i-1) ys(i-1)], obstacles_pos, obstacles_radius, 5, [70 70], 1);
    xs(i) = dx + xs(i-1);
    ys(i) = dy + ys(i-1);
end
plot(xs, ys)
%% 

function [dx, dy] = calculate_movement_direction_and_step(position, obstacles_pos, obstacles_radius, square_side, goal_position, desired_speed)  
     
    dx = 0;  
    dy = 0;  
      
    
    center_position = position + [square_side/2, square_side/2];  
      
    
    attractive_force = 0.5*attractive_potential_gradient(center_position, goal_position);
    repulsive_force = zeros(1, 2);  
    for i = 1:length(obstacles_radius)  
        obstacle_force = repulsive_potential_gradient(center_position, obstacles_pos(:, i)', obstacles_radius(i), square_side, square_side/2);
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