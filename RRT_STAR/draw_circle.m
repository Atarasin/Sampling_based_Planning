function f = draw_circle(x,y,radius)
t = linspace(0, 2*pi);

x = cos(t)*radius+x;
y = sin(t)*radius+y;
f = plot(x,y,'--m'); hold on;

end