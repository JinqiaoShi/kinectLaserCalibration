function newRange=applyCorrection(Vrange,Vangle,k1,k2)
	newRange= Vrange + (Vrange*Vrange*k1) + (Vrange*Vrange*Vangle*Vangle*k1*k2);
endfunction
