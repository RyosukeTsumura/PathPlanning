function [curvature] = BowelMeasure(I)

[row, col]=size(I);

[x,y,P] = impixel(I);
ellipse_t = fit_ellipse(x,y);

t = linspace(0, 2*pi);

syms X Y

curvature = (((X - ellipse_t.X0_in)*cos(-ellipse_t.phi)+(Y - ellipse_t.Y0_in)*sin(-ellipse_t.phi))/ellipse_t.a)^2 + (((-(X - ellipse_t.X0_in))*sin(-ellipse_t.phi)+(Y - ellipse_t.Y0_in)*cos(-ellipse_t.phi))/ellipse_t.b)^2 - 1;

end
