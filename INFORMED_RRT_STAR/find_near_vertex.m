function near_nodes_index = find_near_vertex(new_node, search_radius, step_len, map_config)
% �ҵ�����������Բ�ڵĽڵ�����

global VERTEX;
near_nodes_index = [];
n = length(VERTEX);
% +1����Ϊ�½ڵ㻹û�м��뵽V��
r = min([search_radius*sqrt(log(n+1)/(n+1)),step_len]);
% r = 3;
idx = 1;
for i = 1:n
    dist = norm([new_node.nx, new_node.ny]-[VERTEX{i}.nx, VERTEX{i}.ny]);
    if dist <= r
        if ~map_config.collision_check(VERTEX{i},new_node)
            near_nodes_index(idx) = i;
            idx = idx + 1;
        end
    end
end

end
