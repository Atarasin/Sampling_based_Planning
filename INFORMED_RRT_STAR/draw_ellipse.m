function f = draw_ellipse(x0, y0, a, b, theta)
% 绘制椭圆
% x0,y0为椭圆圆心
% a,b为半长轴和半短轴
% theta为旋转角度，逆时针为正数

theta = theta*(-1);
t = linspace(0, 2*pi);
% 未旋转的坐标
x = cos(t)*a;
y = sin(t)*b;
points = [x' y'];  % 100*2
% 定义旋转矩阵
RotMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
% 旋转后的坐标
points = points * RotMatrix + [x0, y0];

f = plot(points(:,1), points(:,2), '--m'); hold on;

end