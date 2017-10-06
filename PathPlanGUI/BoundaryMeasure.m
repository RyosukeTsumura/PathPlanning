function [curvature] = BoundaryMeasure(I)
    %% symblic
    syms X Y
    
	[row,col]=size(I);

	%% select surface edge
	[x, y, P] = impixel(I);
	ps = sortrows([x,y]);%sort(left->right)
	pol_s= polyfit(ps(:,1),ps(:,2),2);%second order approximation    
	%pol_s= polyfit(ps(:,1),ps(:,2),3);
    curvature = pol_s(1,1)*X^2 + pol_s(1,2)*X+pol_s(1,3) - Y;
    %curvature = pol_s(1,1)*X^3 + pol_s(1,2)*X^2 + pol_s(1,3)*X + pol_s(1,4) - Y;
end
