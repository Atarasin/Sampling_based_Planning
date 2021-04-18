function path = extract_path(start,goal,term_vertex)
% ��ȡ·��
global VERTEX1; global VERTEX2;

% ����VERTEX1�ĵ�һ���ڵ������
if isequal([VERTEX2{1}.nx,VERTEX2{1}.ny],start)
    tmp = VERTEX2;
    VERTEX2 = VERTEX1;
    VERTEX1 = tmp;
end
node_list1 = zeros(length(VERTEX1),2);
for i = 1:size(node_list1,1)
    node_list1(i,:) = [VERTEX1{i}.nx,VERTEX1{i}.ny];
end
node_list2 = zeros(length(VERTEX2),2);
for i = 1:size(node_list2,1)
    node_list2(i,:) = [VERTEX2{i}.nx,VERTEX2{i}.ny];
end

path1 = term_vertex;
vertex = VERTEX1{end}.parent;
while true
    path1 = [path1;vertex];
    if vertex == start
        break;
    end
    [~,index] = ismember(vertex,node_list1,'rows'); 
    vertex = VERTEX1{index}.parent;
end
path1(1,:) = [];  % ɾȥterm_vertex����ֹ�ظ�
path1 = flipud(path1);  % ��ת����

path2 = term_vertex;
vertex = VERTEX2{end}.parent;
while true
    path2 = [path2;vertex];
    if vertex == goal
        break;
    end
    [~,index] = ismember(vertex,node_list2,'rows'); 
    vertex = VERTEX2{index}.parent;
end

path = [path1; path2];

end