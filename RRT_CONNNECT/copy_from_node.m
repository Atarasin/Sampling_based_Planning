function new_node = copy_from_node(node)
 % ��node���Ƶ�new_node
 
 new_node = RrtNode(node.nx,node.ny,node.parent);
 
end