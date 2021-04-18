function node = find_goal_parent(goal, step_len, map_config)
% �ҵ�goal�ĸ��ڵ㣬�����㣺
% (1)��goal����С�ڲ���
% (2)������С

global VERTEX;
goal_node = RrtNode(goal(1),goal(2));
advailable_node = {};
idx = 1;
for i = 1:length(VERTEX)
    dist = norm([VERTEX{i}.nx, VERTEX{i}.ny]-goal);
    if dist < step_len
        % �ų���ײ�Ľڵ�
        if ~map_config.collision_check(VERTEX{i}, goal_node) 
            advailable_node{idx} = VERTEX{i};
            idx = idx + 1;
        end
    end
end
% ��Ϊ����˵��·����δ����
if isempty(advailable_node)
    node = RrtNode(-1,-1);
    return;
end

min_cost = 1000;
for i = 1:length(advailable_node)
    cost = advailable_node{i}.cost + norm([advailable_node{i}.nx, advailable_node{i}.ny]-goal);
    if cost < min_cost
        min_cost = cost;
        node = advailable_node{i};
    end
end
    
end