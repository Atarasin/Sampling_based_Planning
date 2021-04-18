function new_node = set_new_node(new_node,near_node,step_len)
% ���������ڵ�֮��ľ���
new_vertex = [new_node.nx, new_node.ny];
near_vertex = [near_node.nx, near_node.ny];
dist = min([norm(new_vertex-near_vertex),step_len]);
theta = atan2(new_node.ny-near_node.ny,new_node.nx-near_node.nx);

new_node.nx = near_node.nx + dist*cos(theta);
new_node.ny = near_node.ny + dist*sin(theta);
new_node = new_node.set_parent([near_node.nx,near_node.ny]);  % ���ø��ڵ�

end