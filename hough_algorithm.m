scan = csvread('box.csv');
scan = csvread('circle.csv');

% Delta Phi is the angle between each of the readings (convert to rad as
% matlab uses rad).
dPhi = degtorad(180/512);

% create a new box array to hold both r and phi.
scan_clean = zeros(length(scan), 2);
for i = 1:length(scan)
    scan_clean(i, 1) = scan(i);
    scan_clean(i, 2) = dPhi * i;
end

% Convert co-ordinates of rho and phi to x an y frame relative to the
% current laser position.
x = [];
y = [];
multiplier = 200;
count = 1;
for i = 1:length(scan_clean)
    if scan_clean(i, 1) < 5
        x(count, 1) = ceil((scan_clean(i, 1) * cos(scan_clean(i, 2)) + 6) * multiplier); % x co-ordinate
        y(count, 1) = ceil(scan_clean(i, 1) * sin(scan_clean(i, 2)) * multiplier); % y co-ordinate
        count = count + 1;
    end
end

img = zeros(12 * multiplier, 6 * multiplier);
for i = 1:length(x)
    disp([x(i), y(i)])
    img(x(i), y(i)) = 1;
end

% imshow(img)

[H,T,R] = hough(img);
imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;

P  = houghpeaks(H,10,'threshold',ceil(0.3*max(H(:))));
x_plot = T(P(:,2)); y_plot = R(P(:,1));
plot(x_plot,y_plot,'s','color','white');

lines = houghlines(img,T,R,P,'FillGap',0.5 * multiplier/10,'MinLength',2 * multiplier/10);
figure, imshow(img), hold on
max_len = 0;

for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
% p = length(x);
% 
% accumulator = zeros(p, 180);
% rtheta = zeros(20, 180);
% For each x, y coordinate work out the r and place these values in the 
% corresponding theta column.
% for i = 1:p
%     for j = 1:180
%         r = (x(i) * cos(degtorad(j)) + y(i) * sin(degtorad(j))) + 6;
%         accumulator(i, j) = r;
%         
%         % Add the found r to the phi in the accumulator.
%         rtheta(ceil(r) ,j) = rtheta(ceil(r) ,j) + 1;
%     end 
% end

% Plot the two curves.
% plot(1:180, accumulator)

% Plot the points of best fit.
% plot_x = 1:180;
% r point, value
% plot_y = zeros(2, 180);

% for n = 1:length(rtheta)
%     for i = 1:size(rtheta, 1)
%         if plot_y(2, n) < rtheta(i, n)
%             plot_y(1, n) = i;
%             plot_y(2, n) = rtheta(i, n);
%         end
%     end
% end

% plot(plot_x, plot_y(1, :))

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