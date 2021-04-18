classdef RrtNode
    properties
        nx = -1;
        ny = -1;
        parent = [-1,-1];
        cost = 0;
    end
    methods
        % 构造函数
        function node = RrtNode(x,y,varargin)
            node.nx = x;
            node.ny = y;
            if nargin > 2
                node.parent = varargin{1};
            end
        end
        % 设定参数
        function node = set_parent(node,parent)
            node.parent = parent;
        end
    end
end