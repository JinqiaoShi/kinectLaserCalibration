function e=computeError(i,X,Z)
  pi=ones(3,1);
  pj=ones(3,1);
  pi(1:2)=Z(1:2,i);
  pj(1:2)=Z(3:4,i);
  efull = pi-X*pj;
  e=efull(1:2);
endfunction