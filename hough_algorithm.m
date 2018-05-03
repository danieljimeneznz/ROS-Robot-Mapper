box = csvread('box.csv');
circle = csvread('circle.csv');

% Delta Phi is the angle between each of the readings (convert to rad as
% cos uses rad).
dPhi = degtorad(180/512);

% Changing box for testing.
box = [2.04310000000000;1.02260000000000];

% Convert co-ordinates of rho and phi to x an y frame relative to the
% current laser position.
x = zeros(length(box), 1);
y = zeros(length(box), 1);
for i = 1:length(box)
    x(i, 1) = box(i) * cos(dPhi * i); % x co-ordinate
    y(i, 1) = box(i) * sin(dPhi * i); % y co-ordinate
    
    % Add an offset of 6 as this is as far as the scanner can scan.
    % Meaning the actual point is x - 6, y - 6.
%   x(i, 1) = x(i, 1) + 6;
% 	y(i, 1) = y(i, 1) + 6;
end

p = length(x);

accumulator = zeros(p, 180);
rtheta = zeros(20, 180);
% For each x, y coordinate work out the r and place these values in the 
% corresponding theta column.
for i = 1:p
    for j = 1:180
        r = (x(i) * cos(degtorad(j)) + y(i) * sin(degtorad(j))) + 6;
        accumulator(i, j) = r;
        
        % Add the found r to the phi in the accumulator.
        rtheta(ceil(r) ,j) = rtheta(ceil(r) ,j) + 1;
    end 
end

% Plot the two curves.
% plot(1:180, accumulator)

% Plot the points of best fit.
plot_x = 1:180;
% r point, value
plot_y = zeros(2, 180);

for n = 1:length(rtheta)
    for i = 1:size(rtheta, 1)
        if plot_y(2, n) < rtheta(i, n)
            plot_y(1, n) = i;
            plot_y(2, n) = rtheta(i, n);
        end
    end
end

plot(plot_x, plot_y(1, :))

% Inspired by http://www.keymolen.com/2013/05/hough-transformation-c-implementation.html?m=1
% img_w = w;  
% img_h = h;  
% 
% %  Create the accu  
% hough_h = ((sqrt(2.0) * (h>w?h:w)) / 2.0);  
% accu_h = hough_h * 2.0; % -r -> +r  
% accu_w = 180;  
% 
% accu = zeros(accu_h, accu_w);  
% 
% center_x = w/2;  
% center_y = h/2;  
% 
% img_data = [];
% 
% for y = 1:h
%     for x = 1:w
%         if img_data[(y*w) + x] > 250
%             for t = 0:180  
%                 r = ( (x - center_x) * cos(t * DEG2RAD)) + ((y - center_y) * sin(t * DEG2RAD));  
%                 accu[(round(r + hough_h) * 180.0)) + t]++;  
%             end
%         end
%     end
% end