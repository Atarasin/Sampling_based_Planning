%% 1.初始设置
clear
close all
clc
map_config = MapConfig;
global VERTEX1; global VERTEX2
VERTEX1 = {};  % 存储节点
VERTEX2 = {}; 
start = [2,2];
goal = [49,24];
step_len = 0.5;  % 节点扩展的步长
max_iters = 5000;
%% 2.RRT-CONNECT
start_node = RrtNode(start(1),start(2));
VERTEX1{1} = start_node;
goal_node = RrtNode(goal(1),goal(2));
VERTEX2{1} = goal_node;
fig = figure('Name','RRT_CONNECT');
map_config.map_plot(); hold on;
scatter([start(1),goal(1)],[start(2),goal(2)],'r*'); hold on;
idx = 1;
for it = 1:max_iters
    new_node = map_config.random_create_node(goal);
    [new_node,near_node] = find_nearest_node(new_node,VERTEX1);
    % 限制新节点的距离并且设置父节点
    new_node = set_new_node(new_node,near_node,step_len);
    if ~map_config.collision_check(near_node,new_node)
        VERTEX1{length(VERTEX1)+1} = new_node;
        scatter(new_node.nx,new_node.ny,'b.'); hold on;
        drawnow;
%         % 抓取图像
%         frame = getframe(fig); 
%         im{idx}=frame2im(frame);
%         idx = idx + 1;
        % from another tree
        [new_node,near_node_prim] = find_nearest_node(new_node,VERTEX2);
        new_node_prim = set_new_node(new_node,near_node_prim,step_len);
        if ~map_config.collision_check(near_node_prim,new_node_prim)
            VERTEX2{length(VERTEX2)+1} = new_node_prim;
            scatter(new_node_prim.nx,new_node_prim.ny,'b.'); hold on;
            drawnow;
%             % 抓取图像
%             frame = getframe(fig); 
%             im{idx}=frame2im(frame);
%             idx = idx + 1;
            % 贪婪寻点
            while true
                new_node_prim2 = set_new_node(new_node,new_node_prim,step_len);
                if ~map_config.collision_check(new_node_prim,new_node_prim2)
                    VERTEX2{length(VERTEX2)+1} = new_node_prim2;
                    scatter(new_node_prim2.nx,new_node_prim2.ny,'b.'); hold on;
                    drawnow;
%                     % 抓取图像
%                     frame = getframe(fig); 
%                     im{idx}=frame2im(frame);
%                     idx = idx + 1;
                    new_node_prim = copy_from_node(new_node_prim2);
                else
                    break;
                end
                % 判断两树的末端节点是否一致
                if node_same_check(new_node,new_node_prim)
                    break;
                end
            end
            % 终止条件
            if node_same_check(new_node,new_node_prim)
                disp("RRT_CONNECT has already find a path.");
                disp("iters: ");
                disp(it);
                break;
            end
        end
        
        % 交换两个树
        tmp = VERTEX1;
        VERTEX1 = VERTEX2;
        VERTEX2 = tmp;
        
%         frame = getframe(fig); 
%         im{idx}=frame2im(frame);
%         idx = idx + 1;
    end  
end
%% 3.提取路径
term_vertex = [new_node.nx,new_node.ny];
path = extract_path(start,goal,term_vertex);
plot(path(:,1),path(:,2),'gd');
% frame = getframe(fig); 
% im{idx}=frame2im(frame);
% %% 4.制作gif
% filename = 'RRT_CONNECT.gif'; % Specify the output file name
% for idx = 1:length(im)
%     [A,map] = rgb2ind(im{idx},256);
%     if idx == 1
%         imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.1);
%     else
%         imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.1);
%     end
% end