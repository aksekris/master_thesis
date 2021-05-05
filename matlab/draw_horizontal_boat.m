function [] = draw_horizontal_boat(x, y, psi)
%DRAW_HORIZONTAL_BOAT Summary of this function goes here
%   Detailed explanation goes here
length = 0.4;
width = 0.3;
x_1 = length*[-0.6, 0.4, 1, 0.4, -0.6, -0.6];
y_1 = width*[-0.5, -0.5, 0, 0.5, 0.5, -0.5];
x_4 = [0, 0, 0, 0, 0, 0];
y_4 = [0, 0, 0, 0, 0, 0];
x_11 = 0.05*length*[-0.6, 0.4, 1, 0.4, -0.6, -0.6];
y_11 = 0.05*width*[-0.5, -0.5, 0, 0.5, 0.5, -0.5];
for i = 1:6
    yeah = [cos(psi), -sin(psi); sin(psi), cos(psi)]*[x_1(i);y_1(i)];
    x_4(i) = yeah(1) + y;
    y_4(i) = yeah(2) + x;
    yeah = [cos(psi), -sin(psi); sin(psi), cos(psi)]*[x_11(i);y_11(i)];
    x_44(i) = yeah(1) + y;
    y_44(i) = yeah(2) + x;
end
plot(y_4, x_4, 'LineWidth',1, 'Color',[0.75,0,0])
plot(y_44, x_44, 'LineWidth',1, 'Color',[0.75,0,0])
end

