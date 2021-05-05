clc
clear all
close all

T = readtable('path1.csv');

x = T.('x');
y = T.('y');
z = T.('z');
gamma_p = T.('gamma_p');
chi_p = T.('chi_p');

w = zeros(length(x),1);
for i = 1:length(x)-1
    w(i+1) = w(i) + sqrt((x(i)-x(i+1))^2+(y(i)-y(i+1))^2);
    
end

%plot(x, y)
plot(w, z)