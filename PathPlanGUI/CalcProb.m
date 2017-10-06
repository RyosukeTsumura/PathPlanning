function [TarProb] = CalcProb(csvFileName, length)

data = dlmread(csvFileName,',',1,0);

x_values = -2:0.1:10;

for k = 1:5
    %hold on
    pd(1,k) = fitdist(data(:,k),'Normal');
    y(:,k) = pdf(pd(:,k),x_values);
end


%length = 100;
TargetSize = 2;
Threshold = 180/pi()*atan(TargetSize/length);

%fun = @(x, xdata)x(1)*xdata.^3+x(2)*xdata.^2+x(3)*xdata+x(4);
%x0 = [0, 0, 0, 0];
%x = lsqcurvefit(fun,x0,xdata,ydata);
%times = linspace(xdata(1),xdata(end));
%plot(xdata,ydata,'ko',times,fun(x,times),'b-')

%plot(x_values, y(:,:), 'LineWidth', 2)

for i = 1:5
	sigma(i)=pd(i).sigma;
	mu(i)=pd(i).mu;
end

xangle=[0,10,20,30,40];
psigma=polyfit(xangle,sigma,3);
pmu=polyfit(xangle,mu,2);

xangle2=0:1:40;
psigma2=polyval(psigma,xangle2);
pmu2=polyval(pmu,xangle2);

%{
norm = normpdf(x_values,pmu2(25),psigma2(25));
figure;
plot(x_values,norm);

Prob = cumtrapz(x_values,norm);
figure;
plot(x_values,Prob);
index = find(x_values == round(Threshold));
TarProb = Prob(index);
%}

for i=1:41
	norm(i,:)=normpdf(x_values,pmu2(i),psigma2(i));
	Prob(i,:)=cumtrapz(x_values,norm(i,:));
	index=find(x_values==round(Threshold));
	TarProb(i)=Prob(i,index);
end

%figure
%plot([0:1:40],TarProb, 'LineWidth', 2);
%figure
%plot(x_values,Prob(10,:), 'LineWidth', 2);
