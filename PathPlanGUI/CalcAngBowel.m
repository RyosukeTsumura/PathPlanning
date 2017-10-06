% Initialize
clear;
clc;

src = imread('Lower-Abdomen.jpg');
I = rgb2gray(src);
[row, col]=size(I);
[x,y,P] = impixel(I);
ellipse_t = fit_ellipse(x,y);

t = linspace(0, 2*pi);
R = [cos(-ellipse_t.phi) -sin(-ellipse_t.phi); sin(-ellipse_t.phi) cos(-ellipse_t.phi)];
X = ellipse_t.a * cos(t);
Y = ellipse_t.b * sin(t);

Z = R*[X;Y];

X1 = Z(1,:) + ellipse_t.X0_in;
Y1 = Z(2,:) + ellipse_t.Y0_in;

%X = X*cos(ellipse_t.phi) + Y*sin(ellipse_t.phi) + ellipse_t.X0;
%Y = -X*sin(ellipse_t.phi) + Y*cos(ellipse_t.phi) + ellipse_t.Y0;

hold on

syms Xi Yi

bowel = (((Xi - ellipse_t.X0_in)*cos(-ellipse_t.phi)+(Yi - ellipse_t.Y0_in)*sin(-ellipse_t.phi))/ellipse_t.a)^2 + (((-(Xi - ellipse_t.X0_in))*sin(-ellipse_t.phi)+(Yi - ellipse_t.Y0_in)*cos(-ellipse_t.phi))/ellipse_t.b)^2 - 1;

[x,y,P] = impixel(I);

ps = sortrows([x,y]);%sort(left->right)
ps_s = ps(1,:);%start point
ps_e = ps(end,:);%end point
pol_s= polyfit(ps(:,1),ps(:,2),2);%second order approximation
xs = ceil(ps_s(1,1):1:ps_e(1,1));ys = ceil(polyval(pol_s, xs));%surface points
    

for i=1:ceil(size(xs,2))
	ip(i,:) = [xs(i),ys(i)];
end

N = size(ip,1);
for i=1:N
    [a,b,c] = CalcFirstOrder(ip(i,:), [400 300]);
	path(i) = a*Xi + b*Yi + c;
	df_path(i) = -a/b;
end

df_bowel = diff(bowel, Xi);

for i=1:N
	[solX,solY] = solve([bowel == 0,path(i) == 0], [Xi,Yi]);
	solX = abs(solX);solY = abs(solY);
    %solX = sort(abs(solX), 1);solY = sort(abs(solY), 1);
	cp1(i,:) = [double(solX(1)),double(solY(1))];
    cp2(i,:) = [double(solX(2)),double(solY(2))];
	angle1(i) = atan(double(subs(df_bowel, [Xi,Yi], [cp1(i,1),cp1(i,2)])));
	angle1(i) = radtodeg(angle1(i));
    angle2(i) = atan(double(subs(df_bowel, [Xi,Yi], [cp2(i,1),cp2(i,2)])));
	angle2(i) = radtodeg(angle2(i));
end

hold on
plot(X1,Y1,'o');
plot(cp1(:,1),cp1(:,2),'o');
plot(cp2(:,1),cp2(:,2),'o');
plot(ip(:,1),ip(:,2),'o');
plot(400,300,'o');

for i = 1:N
    plot([ip(i,1),400], [ip(i,2), 300],'-');
end
 Initialize
clear;
clc;

src = imread('Lower-Abdomen.jpg');
I = rgb2gray(src);
[row, col]=size(I);
[x,y,P] = impixel(I);
ellipse_t = fit_ellipse(x,y);

t = linspace(0, 2*pi);
R = [cos(-ellipse_t.phi) -sin(-ellipse_t.phi); sin(-ellipse_t.phi) cos(-ellipse_t.phi)];
X = ellipse_t.a * cos(t);
Y = ellipse_t.b * sin(t);

Z = R*[X;Y];

X1 = Z(1,:) + ellipse_t.X0_in;
Y1 = Z(2,:) + ellipse_t.Y0_in;

%X = X*cos(ellipse_t.phi) + Y*sin(ellipse_t.phi) + ellipse_t.X0;
%Y = -X*sin(ellipse_t.phi) + Y*cos(ellipse_t.phi) + ellipse_t.Y0;

hold on

syms Xi Yi

bowel = (((Xi - ellipse_t.X0_in)*cos(-ellipse_t.phi)+(Yi - ellipse_t.Y0_in)*sin(-ellipse_t.phi))/ellipse_t.a)^2 + (((-(Xi - ellipse_t.X0_in))*sin(-ellipse_t.phi)+(Yi - ellipse_t.Y0_in)*cos(-ellipse_t.phi))/ellipse_t.b)^2 - 1;

[x,y,P] = impixel(I);

ps = sortrows([x,y]);%sort(left->right)
ps_s = ps(1,:);%start point
ps_e = ps(end,:);%end point
pol_s= polyfit(ps(:,1),ps(:,2),2);%second order approximation
xs = ceil(ps_s(1,1):1:ps_e(1,1));ys = ceil(polyval(pol_s, xs));%surface points
    

for i=1:ceil(size(xs,2))
	ip(i,:) = [xs(i),ys(i)];
end

N = size(ip,1);
for i=1:N
    [a,b,c] = CalcFirstOrder(ip(i,:), [400 300]);
	path(i) = a*Xi + b*Yi + c;
	df_path(i) = -a/b;
end

df_bowel = diff(bowel, Xi);

for i=1:N
	[solX,solY] = solve([bowel == 0,path(i) == 0], [Xi,Yi]);
	solX = abs(solX);solY = abs(solY);
    %solX = sort(abs(solX), 1);solY = sort(abs(solY), 1);
	cp1(i,:) = [double(solX(1)),double(solY(1))];
    cp2(i,:) = [double(solX(2)),double(solY(2))];
	angle1(i) = atan(double(subs(df_bowel, [Xi,Yi], [cp1(i,1),cp1(i,2)])));
	angle1(i) = radtodeg(angle1(i));
    angle2(i) = atan(double(subs(df_bowel, [Xi,Yi], [cp2(i,1),cp2(i,2)])));
	angle2(i) = radtodeg(angle2(i));
end

hold on
plot(X1,Y1,'o');
plot(cp1(:,1),cp1(:,2),'o');
plot(cp2(:,1),cp2(:,2),'o');
plot(ip(:,1),ip(:,2),'o');
plot(400,300,'o');

for i = 1:N
    plot([ip(i,1),400], [ip(i,2), 300],'-');
end
 Initialize
clear;
clc;

src = imread('Lower-Abdomen.jpg');
I = rgb2gray(src);
[row, col]=size(I);
[x,y,P] = impixel(I);
ellipse_t = fit_ellipse(x,y);

t = linspace(0, 2*pi);
R = [cos(-ellipse_t.phi) -sin(-ellipse_t.phi); sin(-ellipse_t.phi) cos(-ellipse_t.phi)];
X = ellipse_t.a * cos(t);
Y = ellipse_t.b * sin(t);

Z = R*[X;Y];

X1 = Z(1,:) + ellipse_t.X0_in;
Y1 = Z(2,:) + ellipse_t.Y0_in;

%X = X*cos(ellipse_t.phi) + Y*sin(ellipse_t.phi) + ellipse_t.X0;
%Y = -X*sin(ellipse_t.phi) + Y*cos(ellipse_t.phi) + ellipse_t.Y0;

hold on

syms Xi Yi

bowel = (((Xi - ellipse_t.X0_in)*cos(-ellipse_t.phi)+(Yi - ellipse_t.Y0_in)*sin(-ellipse_t.phi))/ellipse_t.a)^2 + (((-(Xi - ellipse_t.X0_in))*sin(-ellipse_t.phi)+(Yi - ellipse_t.Y0_in)*cos(-ellipse_t.phi))/ellipse_t.b)^2 - 1;

[x,y,P] = impixel(I);

ps = sortrows([x,y]);%sort(left->right)
ps_s = ps(1,:);%start point
ps_e = ps(end,:);%end point
pol_s= polyfit(ps(:,1),ps(:,2),2);%second order approximation
xs = ceil(ps_s(1,1):1:ps_e(1,1));ys = ceil(polyval(pol_s, xs));%surface points
    

for i=1:ceil(size(xs,2))
	ip(i,:) = [xs(i),ys(i)];
end

N = size(ip,1);
for i=1:N
    [a,b,c] = CalcFirstOrder(ip(i,:), [400 300]);
	path(i) = a*Xi + b*Yi + c;
	df_path(i) = -a/b;
end

df_bowel = diff(bowel, Xi);

for i=1:N
	[solX,solY] = solve([bowel == 0,path(i) == 0], [Xi,Yi]);
	solX = abs(solX);solY = abs(solY);
    %solX = sort(abs(solX), 1);solY = sort(abs(solY), 1);
	cp1(i,:) = [double(solX(1)),double(solY(1))];
    cp2(i,:) = [double(solX(2)),double(solY(2))];
	angle1(i) = atan(double(subs(df_bowel, [Xi,Yi], [cp1(i,1),cp1(i,2)])));
	angle1(i) = radtodeg(angle1(i));
    angle2(i) = atan(double(subs(df_bowel, [Xi,Yi], [cp2(i,1),cp2(i,2)])));
	angle2(i) = radtodeg(angle2(i));
end

hold on
plot(X1,Y1,'o');
plot(cp1(:,1),cp1(:,2),'o');
plot(cp2(:,1),cp2(:,2),'o');
plot(ip(:,1),ip(:,2),'o');
plot(400,300,'o');

for i = 1:N
    plot([ip(i,1),400], [ip(i,2), 300],'-');
end

