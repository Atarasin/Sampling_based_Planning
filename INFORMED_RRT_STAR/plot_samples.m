clear;
clc;
sample_num = 10000;
map_config = MapConfig;
samples_x = [];
samples_y = [];
for i = 1:sample_num
%     vertex = SampleUnitNBall();
    node = map_config.sample_from_ellipse([1 3], [5, 3], 6);
    samples_x = [samples_x; node.nx];
    samples_y = [samples_y; node.ny];
end
scatter(samples_x, samples_y, 'bo'); hold on;
% draw_circle(0, 0, 1);
axis equal;