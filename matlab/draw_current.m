function [] = draw_current(current_x, current_y)
%DRAW_CURRENT Summary of this function goes here
%   Detailed explanation goes here
quiver(ones(10)*current_x,ones(10)*current_y, 'Color',[0.3,0.6,1])
end

