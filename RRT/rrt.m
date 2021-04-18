%% 1.初始设置
clear
close all
clc
map_config = MapConfig;
global VERTEX;
VERTEX = {};  % 存储节点
start = [2,2];
goal = [49,24];
step_len = 0.5;  % 节点扩展的步长
max_iters = 5000;
%% 2.RRT
start_node = RrtNode(start(1),start(2));
VERTEX{1} = start_node;
fig = figure('Name','RRT');
map_config.map_plot(); hold on;
scatter([start(1),goal(1)],[start(2),goal(2)],'r*'); hold on;
% idx = 1;
for it = 1:max_iters
    new_node = map_config.random_create_node(goal);
    [new_node,near_node] = find_nearest_node(new_node);
    % 限制新节点的距离并且设置父节点
    new_node = set_new_node(new_node,near_node,step_len);
    if ~map_config.collision_check(near_node,new_node)
        VERTEX{length(VERTEX)+1} = new_node;
        d = norm(goal-[new_node.nx,new_node.ny]);
        if d < step_len
            goal_node = RrtNode(goal(1),goal(2),[new_node.nx,new_node.ny]);
            if ~map_config.collision_check(new_node,goal_node)
                VERTEX{length(VERTEX)+1} = goal_node;
                disp("RRT has already find a path.");
                disp("iters: ");
                disp(it);
                break;
            end
        end
        scatter(new_node.nx,new_node.ny,'b.'); hold on;
        drawnow;
%         frame = getframe(fig); 
%         im{idx}=frame2im(frame);
%         idx = idx + 1;
    end  
end
%% 3.提取路径
path = extract_path(start,goal);
plot(path(:,1),path(:,2),'g');
% frame = getframe(fig); 
% im{idx}=frame2im(frame);
% %% 4.制作gif
% filename = 'RRT.gif'; % Specify the output file name
% for idx = 1:length(im)
%     [A,map] = rgb2ind(im{idx},256);
%     if idx == 1
%         imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.1);
%     else
%         imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.1);
%     end
% end