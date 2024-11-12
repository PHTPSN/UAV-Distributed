function force = attractive_potential_gradient(current_position, goal_position)  
    diff = goal_position - current_position;  
    distance = norm(diff);  
    if distance > 0  
        force = diff / distance;  
    else  
        force = [0; 0];  
    end  
end  