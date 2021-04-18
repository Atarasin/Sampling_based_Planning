function f = draw_circle(x0,y0,radius)
t = linspace(0, 2*pi);

x = cos(t)*radius+x0;
y = sin(t)*radius+y0;
f = plot(x,y,'--m'); hold on;

end