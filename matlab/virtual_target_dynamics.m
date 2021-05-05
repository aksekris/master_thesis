clc
clear all
close all

T = readtable('virtual_target_dynamics1.csv');

t = T.('t');
x = T.('x');
y = T.('y');
z = T.('z');
psi = T.('psi');
dot_s = T.('dot_s');



plot(x, y)
hold on
N = length(t);
n = 10;
for i = round(linspace(1,N,n))
    draw_horizontal_boat(y(i), x(i), psi(i))
end
hold off
plot(t, dot_s)



