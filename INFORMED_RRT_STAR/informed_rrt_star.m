%% 1.初始设置
clear
close all
clc
map_config = MapConfig;
global VERTEX;
VERTEX = {};  % 存储节点
X_soln = {};  % 储存终点的父节点
start = [20,10];
goal = [49,24];
step_len = 10;  % 节点扩展的步长
search_radius = 20;
max_iters = 1000;
%% 2.INFORMED-RRT-STAR
start_node = RrtNode(start(1),start(2));
goal_node = RrtNode(goal(1),goal(2));
VERTEX{1} = start_node;
fig = figure('Name', 'INFORMED_RRT_STAR');
map_config.map_plot(); hold on;
scatter([start(1),goal(1)],[start(2),goal(2)],'r*'); hold on;
first = true;
first_ellipse = true;
path_optimization = false;
best_node = start_node;
best_cost = inf;
% idx = 1;
for it = 1:max_iters
    % 更新best_cost
    if ~isempty(X_soln)
        min_cost = 1000;
        for i = 1:length(X_soln)
            cost = X_soln{i}.cost + norm(goal-[X_soln{i}.nx, X_soln{i}.ny]);
            if cost < min_cost
                min_cost = cost;
                best_node = X_soln{i};
            end
        end
        best_cost = min_cost;
    end
    % 绘制椭圆
    if best_cost < inf
        if ~first_ellipse
            delete(f_ellipse);
        end
        first_ellipse = false;
        center = (goal+start) / 2;
        c = norm(goal-start) / 2;
        a = best_cost / 2;
        b = sqrt(a*a-c*c);
        theta = atan2(goal(2)-start(2), goal(1)-start(1));
        f_ellipse = draw_ellipse(center(1), center(2), a, b, theta);
    end
    new_node = map_config.sample_from_ellipse(start, goal, best_cost);
    [new_node, near_node] = find_nearest_node(new_node, VERTEX);
    % 限制新节点的距离并且设置父节点
    new_node = set_new_node(new_node, near_node, step_len);
    if isequal(goal, [new_node.nx, new_node.ny])
        continue;
    end
    if ~map_config.collision_check(near_node,new_node) 
        scatter(new_node.nx,new_node.ny,'b.'); hold on;
        r = min([search_radius*sqrt(log(length(VERTEX)+1)/(length(VERTEX)+1)), step_len]);
%         r = 3;
        f_circle = draw_circle(new_node.nx, new_node.ny, r); hold on;
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
        
        % 检查新节点是否可以成为终点的父节点
        dist = norm(goal-[new_node.nx, new_node.ny]);
        if dist < step_len
            if ~map_config.collision_check(new_node, goal_node) 
                X_soln{length(X_soln)+1} = new_node; 
            end
        end
        
        if mod(it, 50) == 0
            % 若X_soln非空，则说明至少存在一条可行路径
            if ~isempty(X_soln)
                
                if first
                    first = false;
                    path = extract_path(start, goal, [best_node.nx, best_node.ny]);
                    h = plot(path(:,1),path(:,2),'g'); hold on;
                else
                    delete(h);
                    path = extract_path(start, goal, [best_node.nx, best_node.ny]);
                    h = plot(path(:,1),path(:,2),'g'); hold on;
                end
                disp("Informed RRT* has already found a path.");
                disp(it);
                X = sprintf('node:  (%f, %f), cost: %f.',best_node.nx,best_node.ny, best_node.cost);
                disp(X);
            else
                disp("No complete path.");
                disp(it);
            end
        end
        drawnow;
%         pause(0.1)
        delete(f_circle);
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
% filename = 'INFORMED_RRT_STAR.gif'; % Specify the output file name
% for idx = 1:length(im)
%     [A,map] = rgb2ind(im{idx},256);
%     if idx == 1
%         imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.1);
%     else
%         imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.1);
%     end
% end