function [new_node,near_node] = find_nearest_node(new_node,VERTEX)
% 找到一个离新节点最近的节点

min_dist = 1000;
new_vertex = [new_node.nx, new_node.ny];
for i = 1:length(VERTEX)
    vertex = [VERTEX{i}.nx, VERTEX{i}.ny];
    dist = norm(new_vertex-vertex);
    if dist < min_dist
        min_dist = dist;
        near_node = VERTEX{i};
        new_node.parent = vertex;
    end
end

end