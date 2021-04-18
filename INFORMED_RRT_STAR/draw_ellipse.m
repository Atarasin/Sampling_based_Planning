function f = draw_ellipse(x0, y0, a, b, theta)
% ������Բ
% x0,y0Ϊ��ԲԲ��
% a,bΪ�볤��Ͱ����
% thetaΪ��ת�Ƕȣ���ʱ��Ϊ����

theta = theta*(-1);
t = linspace(0, 2*pi);
% δ��ת������
x = cos(t)*a;
y = sin(t)*b;
points = [x' y'];  % 100*2
% ������ת����
RotMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
% ��ת�������
points = points * RotMatrix + [x0, y0];

f = plot(points(:,1), points(:,2), '--m'); hold on;

end