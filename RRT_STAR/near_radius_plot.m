% 相邻节点选择圆的半径变化图
node_num = 1000;
step_len = 1;
search_radius = 10;
x = 1:1:node_num;
y = zeros(1,node_num);
for i = 1:node_num
    y(i) = min([search_radius*sqrt(log(x(i))/x(i)),step_len]);
end
plot(x,y); 