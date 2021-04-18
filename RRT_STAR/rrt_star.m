%% 1.初始设置
clear
close all
clc
map_config = MapConfig;
global VERTEX;
VERTEX = {};  % 存储节点
start = [2,2];
goal = [49,24];
step_len = 10;  % 节点扩展的步长
search_radius = 20;
max_iters = 1000;
%% 2.RRT-STAR
start_node = RrtNode(start(1),start(2));
VERTEX{1} = start_node;
fig = figure('Name','RRT_STAR');
map_config.map_plot(); hold on;
scatter([start(1),goal(1)],[start(2),goal(2)],'r*'); hold on;
first = true;
path_optimization = false;
% idx = 1;
for it = 1:max_iters
    new_node = map_config.random_create_node(goal);
    [new_node,near_node] = find_nearest_node(new_node,VERTEX);
    % 限制新节点的距离并且设置父节点
    new_node = set_new_node(new_node,near_node,step_len);
    if isequal(goal, [new_node.nx, new_node.ny])
        continue;
    end
    if ~map_config.collision_check(near_node,new_node) 
        scatter(new_node.nx,new_node.ny,'b.'); hold on;
        r = min([search_radius*sqrt(log(length(VERTEX)+1)/(length(VERTEX)+1)), step_len]);
%         r = 3;
        f = draw_circle(new_node.nx, new_node.ny, r); hold on;
        % 寻找新节点的所有相邻节点
        near_nodes_index = find_near_vertex(new_node, search_radius, step_len, map_config);
%         VERTEX{length(VERTEX)+1} = new_node; 
        % Connect along a minimun-cost path
        min_cost = 1000;
        near_node_num = length(near_nodes_index);
        if near_node_num > 0
            for i = 1:length(near_nodes_index)
                index = near_nodes_index(i);
                cost = VERTEX{index}.cost + norm([new_node.nx,new_node.ny]-[VERTEX{index}.nx, VERTEX{index}.ny]);
                if cost < min_cost
                    min_cost = cost;
                    new_node.parent = [VERTEX{index}.nx, VERTEX{index}.ny];
                    new_node.cost = cost;
                end
            end
            % Rewire the tree
            for i = 1:length(near_nodes_index)
                index = near_nodes_index(i);
                cost = new_node.cost + norm([new_node.nx,new_node.ny]-[VERTEX{index}.nx, VERTEX{index}.ny]);
                if cost < VERTEX{index}.cost
                    VERTEX{index}.parent = [new_node.nx, new_node.ny];
                    VERTEX{index}.cost = cost;
                end
            end
        end
        VERTEX{length(VERTEX)+1} = new_node; 
        
        if mod(it, 50) == 0
            node = find_goal_parent(goal, step_len, map_config);
            if isequal([-1,-1], [node.nx, node.ny])
                disp("No complete path.");
                disp(it);
                drawnow;
%                 pause(0.1)
                delete(f);
                continue;
            end
            if first
                first = false;
                path = extract_path(start, goal, [node.nx, node.ny]);
                h = plot(path(:,1),path(:,2),'g'); hold on;
            else
                delete(h);
                path = extract_path(start, goal, [node.nx, node.ny]);
                h = plot(path(:,1),path(:,2),'g'); hold on;
            end
            disp("RRT* has already found a path.");
            disp(it);
            X = sprintf('node:  (%f, %f), cost: %f.',node.nx,node.ny, node.cost);
            disp(X);
        end
        drawnow;
%         pause(0.1)
        delete(f);
        scatter(new_node.nx,new_node.ny,'b.'); hold on;
        drawnow;
%         frame = getframe(fig); 
%         im{idx}=frame2im(frame);
%         idx = idx + 1;
    end 
    
end
%% 3.提取路径
% path = extract_path(start,goal);
% plot(path(:,1),path(:,2),'g');
% frame = getframe(fig); 
% im{idx}=frame2im(frame);
% %% 3.制作gif
% filename = 'RRT_STAR.gif'; % Specify the output file name
% for idx = 1:length(im)
%     [A,map] = rgb2ind(im{idx},256);
%     if idx == 1
%         imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.1);
%     else
%         imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.1);
%     end
% end