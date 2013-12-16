function chi2=pointAlignerChi2(X,Z)
  chi2=0;
  #loop through the measurements and update the
 #accumulators
  for i=1:size(Z,2),
    e=computeError(i,X,Z);
    chi2+=e'*e;
  end
endfunction
