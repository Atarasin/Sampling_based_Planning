function path = extract_path(start, goal, end_vertex)
% ��ȡ·��
% end_vertex��goal�ĸ��ڵ�

global VERTEX;
node_list = zeros(length(VERTEX),2);
for i = 1:size(node_list,1)
    node_list(i,:) = [VERTEX{i}.nx,VERTEX{i}.ny];
end
path = [goal; end_vertex];
[~,index] = ismember(end_vertex,node_list,'rows'); 
vertex = VERTEX{index}.parent;
while true
    path = [path;vertex];
    if vertex == start
        break;
    end
    [~,index] = ismember(vertex,node_list,'rows'); 
    vertex = VERTEX{index}.parent;
end

end