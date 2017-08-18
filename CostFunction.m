function CostFunction()
    close all
    ITER = 11
    % let goal be lane 2
    GOAL = [300.01 2] % goal distance and lane
    goal_lane = GOAL(2)
    goal_dist = GOAL(1)
    max_speed = 50
    lane_0 = 0
    lane_1 = 1
    lane_2 = 2
    % let each lane be 4m wide
    lane_width = 4
    delta_s = linspace(0.0,300,ITER)
    delta_d_l0 = linspace((goal_lane-lane_0)*lane_width,(goal_lane-lane_0)*lane_width,ITER)
    delta_d_l1 = linspace((goal_lane-lane_1)*lane_width,(goal_lane-lane_1)*lane_width,ITER)
    delta_d_l2 = linspace((goal_lane-lane_2)*lane_width,(goal_lane-lane_2)*lane_width,ITER)
    
    % cost function depends on speed, distance to goal, distance to target
    % cost = 1 - exp(-delta_d/delta_s)
    
    current_speed = max_speed;
    cost_lane_0 = 1 - exp(-delta_d_l0./(goal_dist-delta_s))
    cost_lane_1 = 1 - exp(-delta_d_l1./(goal_dist-delta_s))
    cost_lane_2 = 1 - exp(-delta_d_l2./(goal_dist-delta_s))
    
    current_speed = max_speed;
    cost_lane_0 = 1-exp(-1/(goal_dist-delta_s)/goal_dist.*(1-(max_speed-current_speed)/max_speed))
    cost_lane_1 = 1-exp(-1/(goal_dist-delta_s)/goal_dist.*(1-(max_speed-current_speed/2)/max_speed))
    cost_lane_2 = 1-exp(-1/(goal_dist-delta_s)/goal_dist.*(1-(max_speed-0)/max_speed))
    
    figure(1)
    hold on;
    plot (delta_s,cost_lane_0,'-b')
    plot (delta_s,cost_lane_1,'-r')
    plot (delta_s,cost_lane_2,'-g')
    legend('lane 0, v=max','lane 1,v=med','lane 2,v=min')
    xlabel('distance')
    ylabel('cost')
end