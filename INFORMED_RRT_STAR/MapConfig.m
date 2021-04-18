classdef MapConfig
    %% ��������
    properties
        % ��ͼ��С
        map_size = [50, 30];
        % �ϰ�������
        % ÿ�д���һ���ϰ��ÿ���ϰ����ɶ�������ʾ
        obs_boundary = {{[0,0; 1,0; 1,30; 0,30]}, ...
                        {[0,30; 50,30; 50,31; 0,31]}, ...
                        {[1,0; 51,0; 51,1; 1,1]}, ...
                        {[50,1; 51,1; 51,31; 50,31]}
                        };
        obs_polygon = {{[14,12; 22,12; 22,14; 18,16; 14,14]}, ...
                       {[18,22; 22,20; 26,22; 26,25; 18,25]}, ...
                       {[26,7; 28,7; 30,13; 28,19; 26,19]}, ...
                       {[32,14; 42,14; 42,16; 32,16]}
                       };
        % (x,y,r)
        obs_cir = [7, 12, 3;
                   46, 20, 2;
                   15, 5, 2;
                   37, 7, 3;
                   37, 23, 3
                  ];
    end
    %% ���徲̬����
    methods(Static)
        % ---------------------------collision_check------------------------------
        function result = cross_2d(vec1,vec2)
            % �����ά�����Ĳ��
            result = vec1(1)*vec2(2)-vec1(2)*vec2(1);
        end
        function is_left = left_check(A,B,P)
            % ����P������AB��λ�ù�ϵ
            AB = B - A;
            AP = P - A;
            result = MapConfig.cross_2d(AB,AP);  % ��ά�����Ĳ�� x1y2-x2y1
            if result > 0
                is_left = true;
                return;
            % ��Pλ��AB����Ϊ���ϰ�����
            else 
                is_left = false;
                return;
            end
        end
        function is_intersect = intersect_check(A,B,V1,V2)
            % ����߶�AB���߶�V12�Ƿ��ཻ
            % �ж�V1,V2���߶�AB��λ��
            AB = B - A;
            AV1 = V1 - A;
            AV2 = V2 - A;
            result1 = MapConfig.cross_2d(AB,AV1);
            result2 = MapConfig.cross_2d(AB,AV2);
            if result1~=0 && result2~=0
                % ͬ��
                if result1*result2 > 0
                    is_intersect = false;
                    return;
                end
            % ���߶ι���(result1=result2=0)��Ϊ���ཻ
            % ĳ�����߶���(result1=0 || result2=0)��Ϊ���ཻ
            else
                is_intersect = false;
                return;    
            end
            % �ж�A,B���߶�V12��λ��
            V12 = V2 - V1;
            V1A = A - V1;
            V1B = B - V1;
            result1 = MapConfig.cross_2d(V12,V1A);
            result2 = MapConfig.cross_2d(V12,V1B);
            if result1~=0 && result2~=0
                % ͬ��
                if result1*result2 > 0
                    is_intersect = false;
                    return;
                end
            % ���߶ι���(result1=result2=0)��Ϊ���ཻ
            % ĳ�����߶���(result1=0 || result2=0)��Ϊ���ཻ
            else
                is_intersect = false;
                return;    
            end
            is_intersect = true;
        end
        % ---------------------------sample_from_ellipse------------------------------
        function C = RotationToWorldFrame(start, goal)
            % ������Բ����ת����
            dist = norm(goal-start);
            a1 = [(goal-start)'/dist; 0];
            l1 = [1 0 0];
            M = a1 * l1;
            % ����ֵ�ֽ�
            [U, ~, V] = svd(M);
            C = U * diag([1,1,det(U)*det(V)]) * V';
        end
        function vertex = SampleUnitNBall()
            % ����һ���ڵ�λԲ�о��Ȳ����ĵ�
            while true
                x = rand()*2-1;  % (-1,1)�ھ��Ȳ���
                y = rand()*2-1;
                if x*x+y*y < 1
                    break;
                end
            end
            vertex = [x; y; 0];
        end
    end
    %% ������ͨ����
    methods
        function map_plot(map)
            % ���ӻ�map
            % 1.boundary
            for i = 1:size(map.obs_boundary,2)
                % per obs
                vertexs = map.obs_boundary{i}{1};
                fill(vertexs(:,1),vertexs(:,2),'k'); hold on;
            end
            % 2.polygon
            for i = 1:size(map.obs_polygon,2)
                % per obs
                vertexs = map.obs_polygon{i}{1};
                fill(vertexs(:,1),vertexs(:,2),'k'); hold on;
            end
            % 3.circle
            t = linspace(0, 2*pi);
            for i = 1:length(map.obs_cir)
                % per obs
                x = cos(t)*map.obs_cir(i,3)+map.obs_cir(i,1);
                y = sin(t)*map.obs_cir(i,3)+map.obs_cir(i,2);
                fill(x,y,'k'); hold on;
            end
            axis([0 map.map_size(1)+1 0 map.map_size(2)+1]);
        end 
        function node = random_create_node(map,goal)
            % �����50*30��ͼ������һ���ڵ�
            r = rand();
            if r < 0.90
                rx = rand()*map.map_size(1);
                ry = rand()*map.map_size(2);
                node = RrtNode(rx,ry);
            else
                node = RrtNode(goal(1),goal(2));
            end
        end
        function node = sample_from_ellipse(map, start, goal, cmax)
            % ����Բ�о��Ȳ���
            if cmax < inf
                while true
                    cmin = norm(goal-start);
                    center = [(start+goal)' / 2; 0];
                    % ������ת����(n*n)
                    C = MapConfig.RotationToWorldFrame(start, goal);
                    % ����L(n*n)
                    r1 = cmax / 2;
                    r2 = sqrt(cmax*cmax-cmin*cmin) / 2;
                    L = diag([r1, r2, r2]);
                    % �ӵ�λ���о��Ȳ���(1*n)
                    sample_ball = MapConfig.SampleUnitNBall();
                    vertex = C * L * sample_ball + center;
                    if vertex(1)>0 && vertex(1)<map.map_size(1)
                        if vertex(2)>0 && vertex(2)<map.map_size(2)
                            node = RrtNode(vertex(1), vertex(2));
                            return;
                        end
                    end
                end
            else
                node = map.random_create_node(goal);
            end
        end
        function is_in_obs = in_obs_check(map,new_vertex)
            % ���ڵ��Ƿ����ϰ����ڲ�
            is_in_obs = false;
            % 1.boundary
            for i = 1:size(map.obs_boundary,2)
                % per obs
                vertexs = map.obs_boundary{i}{1};
                vertex_num = size(vertexs,1);
                is_all_left = true;
                for j = 1:vertex_num
                    if j == vertex_num
                        is_left = MapConfig.left_check(vertexs(j,:),vertexs(1,:),new_vertex);
                    else
                        is_left = MapConfig.left_check(vertexs(j,:),vertexs(j+1,:),new_vertex);
                    end
                    if ~is_left
                        is_all_left = false; 
                    end
                end
                % ��������ڸ��ϰ������бߵ����
                if is_all_left
                    is_in_obs = true;
                    return;
                end
            end
            % 2.polygon
            for i = 1:size(map.obs_polygon,2)
                % per obs
                vertexs = map.obs_polygon{i}{1};
                vertex_num = size(vertexs,1);
                is_all_left = true;
                for j = 1:vertex_num
                    if j == vertex_num
                        is_left = MapConfig.left_check(vertexs(j,:),vertexs(1,:),new_vertex);
                    else
                        is_left = MapConfig.left_check(vertexs(j,:),vertexs(j+1,:),new_vertex);
                    end
                    if ~is_left
                        is_all_left = false; 
                    end
                end
                % ��������ڸ��ϰ������бߵ����
                if is_all_left
                    is_in_obs = true;
                    return;
                end
            end
            % 3.circle
            for i = 1:size(map.obs_cir,1)
                d = norm(new_vertex-map.obs_cir(i,1:2));
                if d < map.obs_cir(i,3)
                    is_in_obs = true;
                end
            end
        end
        function is_collision = collision_check(map,start_node,goal_node)
            % ��ײ���
            is_collision = false;
            start = [start_node.nx, start_node.ny];
            goal = [goal_node.nx, goal_node.ny];
            % ���˵��Ƿ����ϰ�����
            if map.in_obs_check(start) || map.in_obs_check(goal)
                is_collision = true;
                return;
            end
            % ����߶��Ƿ����ϰ����ཻ
            % 1.boundary
            for i = 1:size(map.obs_boundary,2)
                % per obs
                vertexs = map.obs_boundary{i}{1};
                vertex_num = size(vertexs,1);
                for j = 1:vertex_num
                    if j == vertex_num
                        is_intersect = MapConfig.intersect_check(vertexs(j,:),vertexs(1,:),start,goal);
                    else
                        is_intersect = MapConfig.intersect_check(vertexs(j,:),vertexs(j+1,:),start,goal);
                    end
                    if is_intersect
                        is_collision = true; 
                        return;
                    end
                end
            end
            % 2.polygon
            for i = 1:size(map.obs_polygon,2)
                % per obs
                vertexs = map.obs_polygon{i}{1};
                vertex_num = size(vertexs,1);
                for j = 1:vertex_num
                    if j == vertex_num
                        is_intersect = MapConfig.intersect_check(vertexs(j,:),vertexs(1,:),start,goal);
                    else
                        is_intersect = MapConfig.intersect_check(vertexs(j,:),vertexs(j+1,:),start,goal);
                    end
                    if is_intersect
                        is_collision = true; 
                        return;
                    end
                end
            end
            % 3.circle
            for i = 1:size(map.obs_cir,1)
                lambda = dot(map.obs_cir(i,1:2)-start,goal-start) / (norm(goal-start)*norm(goal-start));
                if lambda < 1 && lambda > 0
                    g = start + lambda*(goal-start);
                    d = norm(g-map.obs_cir(i,1:2));         
                    if d < map.obs_cir(i,3)
                        is_collision = true;
                    end
                end
            end
        end
    end
end