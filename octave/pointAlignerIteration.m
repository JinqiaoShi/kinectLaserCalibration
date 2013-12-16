function [xNew, chi]=pointAlignerIteration(x,Z)
  H=zeros(3,3);
  b=zeros(3,1);
  X=v2t(x); 
  chi=0;
  #loop through the measurements and update the
   #accumulators
  for i=1:size(Z,2),
    e=computeError(i,X,Z)
    J=computeJacobian(i,X,Z);
    H+=J'*J;
    b+=J'*e;
    chi+=e'*e;
  end
  dx=-H\b;
  H
  dX = v2t(dx);
  dx
  dX
  xNew = t2v(dX*X);
endfunction
