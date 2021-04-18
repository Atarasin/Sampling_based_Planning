function is_same = node_same_check(node1,node2)
% 判断两树的节点是否相同
is_same = false;
if (node1.nx == node2.nx) && (node1.ny == node2.ny)
    is_same = true;

end