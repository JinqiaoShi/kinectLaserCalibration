function [x,y] = polarToCartesian(Vrange,Vangle)
	x=Vrange*cos(Vangle);
	y=Vrange*sin(Vangle);
endfunction
