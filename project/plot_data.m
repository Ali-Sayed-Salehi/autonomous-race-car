
% delete all variables
clear;

% close existing figure windows
close ALL;

% load array from file into matlab
% note that columns can be separated by space / tab or commas in matlab
A = load('output.txt');


% time,y1,y2,y3,y4,wb,wf

k = 1;
t  = A(:,k); k = k + 1;

y1 = A(:,k); k = k + 1;
y2 = A(:,k); k = k + 1;
y3 = A(:,k); k = k + 1;

% y4 = A(:,k); k = k + 1;

% wb = A(:,k); k = k + 1;
% wf = A(:,k); k = k + 1;

k = 1;

figure(k); k = k + 1;
plot(t,y1);
ylabel('speed-vf (m/s)');
xlabel('time (s)');


figure(k); k = k + 1;
plot(t,y2);
ylabel('Slip Ratio');
xlabel('time (s)');

figure(k); k = k + 1;
plot(t,y3);
ylabel('Motor Voltage (V)');
xlabel('time (s)');

% figure(k); k = k + 1;
% plot(t,y4);
% ylabel('y4, angle_car_road (rad)');
% xlabel('time (s)');

% figure(k); k = k + 1;
% plot(t,wb);
% ylabel('back wheel velocity, wb (rad/s)');
% xlabel('time (s)');

% figure(k); k = k + 1;
% plot(t,wf);
% ylabel('forward wheel velocity, wf (rad/s)');
% xlabel('time (s)');

% figure(k); k = k + 1;
% plot(t,wb,t,wf,'r');
% ylabel('wheel velocity (rad/s)');
% xlabel('time (s)');
% legend('back wheel velocity wb','forward wheel velocity wf');