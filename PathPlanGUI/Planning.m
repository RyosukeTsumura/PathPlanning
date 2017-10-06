function [OptPos_x, OptPos_y] = Planning(srcSize, ip, tp, skin, muscle, membrane, bowel)
%% symblic
syms X Y

%% insertion path
N = size(ip,1);
for i=1:N
    [a,b,c] = CalcFirstOrder(ip(i,:), tp);
	path(i) = a*X + b*Y + c;
	df_path(i) = -a/b;
end

%% differential boundary curvature
df_skin = diff(skin, X);
df_muscle = diff(muscle, X);
df_membrane = diff(membrane, X);
df_bowel = diff(bowel, X);


%% Calcuration cross point and angle
for i=1:N
	% surface
	[solX,solY] = solve([skin == 0,path(i) == 0], [X,Y]);
	solX = sort(abs(solX), 1);solY = sort(abs(solY), 1);
	cp1(i,:) = [double(solX(1)),double(solY(1))];
	angle1(i) = atan(double(subs(df_skin, X, cp1(i,1))));
	angle1(i) = radtodeg(angle1(i));

	% muscle
	[solX,solY] = solve([muscle == 0,path(i) == 0], [X,Y]);
	solX = sort(abs(solX), 1);solY = sort(abs(solY), 1);
	cp2(i,:) = [double(solX(1)),double(solY(1))];
	angle2(i) = atan(double(subs(df_muscle, X, cp2(i,1))));
	angle2(i) = radtodeg(angle2(i));

	% membrane
	[solX,solY] = solve([membrane == 0,path(i) == 0], [X,Y]);
	solX = sort(abs(solX), 1);solY = sort(abs(solY), 1);
	cp3(i,:) = [double(solX(1)),double(solY(1))];
	angle3(i) = atan(double(subs(df_membrane, X, cp3(i,1))));
	angle3(i) = radtodeg(angle3(i));
  % bowel
	[solX,solY] = solve([bowel == 0,path(i) == 0], [X,Y]);
	solX = abs(solX);solY = abs(solY);
	cp4(i,:) = [double(solX(1)),double(solY(1))];
  cp5(i,:) = [double(solX(2)),double(solY(2))];
	angle4(i) = atan(double(subs(df_bowel, [X,Y], [cp4(i,1),cp4(i,2)])));
	angle4(i) = radtodeg(angle4(i));
	angle5(i) = atan(double(subs(df_bowel, [X,Y], [cp5(i,1),cp5(i,2)])));
	angle5(i) = radtodeg(angle5(i));
  % path angle
	angle0(i) = radtodeg(atan(df_path(i)));
	if(angle0(i)<0)
		angle0(i) = angle0(i)+180;
    end
    
  % each insertion angle
	delta_ang(i,1) = 90-(angle0(i)-angle1(i));
  delta_ang(i,2) = 90-(angle0(i)-angle2(i));
	delta_ang(i,3) = 90-(angle0(i)-angle3(i));
	delta_ang(i,4) = 90-(angle0(i)-angle4(i));
	delta_ang(i,5) = 90-(angle0(i)-angle5(i));
  % distance
	Ltar(i) = pdist2(cp1(i,:),tp);
  L(i,1) = pdist2(cp1(i,:),cp2(i,:));
	L(i,2) = pdist2(cp1(i,:),cp3(i,:));
	L(i,3) = pdist2(cp1(i,:),cp4(i,:));
	L(i,4) = pdist2(cp1(i,:),cp5(i,:));
end

%% Optimization of insertion point  
% square sum of isnertion angle
for i=1:N
    Prob(1,:)=CalcProb('Testdata50.csv',Ltar(i));
    Prob(2,:)=CalcProb('Testdata75.csv',(Ltar(i)-L(i,1)));
    Prob(3,:)=CalcProb('Testdata50.csv',(Ltar(i)-L(i,2)));
    Prob(4,:)=CalcProb('Testdata50.csv',(Ltar(i)-L(i,3)));
    Prob(5,:)=CalcProb('Testdata50.csv',(Ltar(i)-L(i,4)));
    SumAng3(i,1) = abs(delta_ang(i,1)*(1-Prob(1,abs(round(delta_ang(i,1)))+1)) + ((Ltar(i)-L(i,1))/Ltar(i))*delta_ang(i,2)*(1-Prob(2,abs(round(delta_ang(i,2)))+1)) + ((Ltar(i)-L(i,2))/Ltar(i))*delta_ang(i,3)*(1-Prob(3,abs(round(delta_ang(i,3)))+1)) + ((Ltar(i)-L(i,3))/Ltar(i))*delta_ang(i,4)*(1-Prob(4,abs(round(delta_ang(i,4)))+1)) + ((Ltar(i)-L(i,4))/Ltar(i))*delta_ang(i,5)*(1-Prob(5,abs(round(delta_ang(i,5)))+1)));
	%SumAng3(i,1) = delta_ang(i,1)*sin(delta_ang(i,1)) + delta_ang(i,2)*(1-L(i,1)/Ltar)*sin(delta_ang(i,2)) + delta_ang(i,3)*(1-L(i,2)/Ltar)*sin(delta_ang(i,3));
end

[M3, id3] = min(abs(SumAng3));

%% Optimization position
OptPos_x = ip(id3,1);
OptPos_y = ip(id3,2);

%{
%% Show plot
plot(ip(:,1),srcSize(1,1)-ip(:,2));
hold on
%plot(xb1, srcSize(1,1)-yb1);
%plot(xb2, srcSize(1,1)-yb2);
plot(tp(1,1),srcSize(1,1)-tp(1,2),'o','MarkerSize',20);
for i=1:N
	plot(cp1(i,1),srcSize(1,1)-cp1(i,2),'o');
	plot(cp2(i,1),srcSize(1,1)-cp2(i,2),'o');
	plot(cp3(i,1),srcSize(1,1)-cp3(i,2),'o');
end
%plot(ip(id1,1),row-ip(id1,2),'*','MarkerSize',20);
%plot(ip(id2,1),row-ip(id2,2),'o','MarkerSize',20);
plot(ip(id3,1),srcSize(1,1)-ip(id3,2),'+','MarkerSize',20);

axis equal
xlim([0 1000]);
ylim([0 650]);
%}
end
