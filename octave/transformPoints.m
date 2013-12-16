function Pt=transformPoints(X,P)
  Pfull = zeros(3,size(P)(2));
  Pfull(1:2,:)=P;
  Pfull(3,:)=1;
  Pfull=X*Pfull;
  Pt=Pfull(1:2,:);
endfunction