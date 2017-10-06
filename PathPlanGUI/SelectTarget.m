function [target] = SelectTarget(I)
	[x, y, P] = impixel(I);
	target = [x(end),y(end)];
end
