function [xes,chis,xnew]=pointAlignerLoop(x,Z, iterations)
  xes=zeros(3,iterations);
  chis=zeros(1,iterations);
  xnew=x;
  for i=1:iterations
    [xnew,chiNew]=pointAlignerIteration(xnew,Z);
    xes(:,i)=xnew;
    chis(1,i)=chiNew;
  end
endfunction
