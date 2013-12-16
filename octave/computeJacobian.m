function J=computeJacobian(i,X,Z)
  J=zeros(2,3);
  J(1:2,1:2)=-eye(2)
  c=X(1,1);
  s=X(2,1);
  Rprime=[0, -1; 1, 0]; # derivative of the rotation matrix w.r.t theta, theta=0
  pj=Z(3:4,i);
  pj = X(1:2,1:2)*pj+X(1:2,3); # remap pj according to X
  J(1:2,3)=-Rprime * pj;
endfunction;
