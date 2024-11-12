function force = repulsive_potential_gradient(current_position, obstacles_pos, obstacles_radius, square_radius, square_side_half)  
    diff = obstacles_pos - current_position;
    distance = norm(diff);
      
    
    if distance < 0.5*(obstacles_radius + square_radius + square_side_half) 
        
        
        rho = 1 / (distance-0.5*obstacles_radius); 
        force = rho * diff / distance;
    else  
        force = [0 0];  
    end  
end