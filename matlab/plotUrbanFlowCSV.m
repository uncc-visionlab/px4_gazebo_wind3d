clear; close all; clc;

data = csvread('output.csv',1,0);
x1 = data(:,1);
x2 = data(:,2);
x3 = data(:,3);
u1 = data(:,4);
u2 = data(:,5);
u3 = data(:,6);

figure;
subplot(1,2,1)
plot3(x1,x2,x3,'.','MarkerSize',5)
grid on;
view(3)
xlabel('x1');
ylabel('x2')
zlabel('x3')
axis equal;

subplot(1,2,2)
plot3(x1,x2,x3,'.','MarkerSize',1)
grid on;
view(2)
xlabel('x1');
ylabel('x2')
zlabel('x3')
axis equal;

figure;
scaleFactor = 2;
skip = 50;
% downsample
x1 = x1(1:skip:end);
x2 = x2(1:skip:end);
x3 = x3(1:skip:end);
u1 = u1(1:skip:end);
u2 = u2(1:skip:end);
u3 = u3(1:skip:end);

quiver3(x1,x2,x3,u1,u2,u3,scaleFactor,'Color','r')
axis equal
[x1Sorted, iX1s] = sort(x1);
[x2Sorted, iX2s] = sort(x2);
[x3Sorted, iX3s] = sort(x3);
dx1=diff(x1Sorted);
dx1(dx1==0)=[];
dx1(dx1>2*mean(dx1))=[];
subplot(1,3,1), hist(dx1,100), title('dx1 histogram');
dx2=diff(x2Sorted);
dx2(dx2==0)=[];
dx2(dx2>2*mean(dx2))=[];
subplot(1,3,2), hist(dx2,100), title('dx2 histogram');
dx3=diff(x3Sorted);
dx3(dx3==0)=[];
dx3(dx3>2*mean(dx3))=[];
subplot(1,3,3), hist(dx3,100), title('dx3 histogram');
xyz_bounds = [min(x1) max(x1); min(x2) max(x2); min(x3) max(x3)];
x1_delta = (max(x1)-min(x1))/mean(dx1);
x2_delta = (max(x2)-min(x2))/mean(dx2);
x3_delta = (max(x3)-min(x3))/mean(dx3);
numelems = [(max(x1)-min(x1))/x1_delta, ...
    (max(x2)-min(x2))/x2_delta, ...
    (max(x3)-min(x3))/x3_delta];
[x1_grid, x2_grid, x3_grid] = meshgrid(...
    min(x1):mean(dx1):max(x1), ...
    min(x2):mean(dx2):max(x2), ...
    min(x3):mean(dx3):max(x3))
    
u1_grid = interp3(x1,x2,x3,u1,Xq,Yq,Zq)