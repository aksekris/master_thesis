clc
clear all
close all

T = readtable('auv_simulator_without_constraints.csv');
t = T.('t');
x = T.('x');
y = T.('y');
z = T.('z');
phi = T.('phi');
theta = T.('theta');
psi = T.('psi');
u = [T.('u1'), T.('u2'), T.('u3'), T.('u4'), T.('u5'), T.('u6'), T.('u7'), T.('u8')];
u_max = T.('u_max');
u_min = T.('u_min');

plot(y, x, 'Color',[0,0,0])
hold on
N = length(t);
n = 10;
for i = round(linspace(1,N,n))
    draw_horizontal_boat(y(i), x(i), psi(i))
end
%draw_current(1, 0.5)
hold on

%axis([-1 25 -1 25])
title('Generated trajectory viewed in the horizontal plane','fontsize',16,'interpreter','latex')
xlabel('$y_d$','fontsize',14,'interpreter','latex')
ylabel('$x_d$','fontsize',14,'interpreter','latex')




%plot(t,u)
%plot(t, psi)
%plot(t, T.('dot_r'))
%draw_horizontal_boat(1, 1, 0)

%{
plot(t, u)
hold on
plot(t, [u_max, u_min])
%}



