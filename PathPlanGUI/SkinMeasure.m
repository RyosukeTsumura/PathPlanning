function [curvature, ip] = SkinMeasure(I)
    %% symblic
    syms X Y
    
	[row,col]=size(I);

	%% select surface edge
	[x, y, P] = impixel(I);
	ps = sortrows([x,y]);%sort(left->right)
	ps_s = ps(1,:);%start point
	ps_e = ps(end,:);%end point
	pol_s= polyfit(ps(:,1),ps(:,2),2);%second order approximation
	xs = ceil(ps_s(1,1):1:ps_e(1,1));ys = ceil(polyval(pol_s, xs));%surface points
    
    SmpNum = 10;
	for i=1:ceil(size(xs,2))
		ip(i,:) = [xs(i),ys(i)];
    end
    ip = [decimate(ip(:,1),SmpNum),decimate(ip(:,2),SmpNum)];%downsampling insertion point number
	curvature = pol_s(1,1)*X^2 + pol_s(1,2)*X+pol_s(1,3) - Y;
end
