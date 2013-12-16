laser=load("laserdataset.m");


%measurement matrix
Z=zeros(4,size(laser,1));
Z(1:2,:)=laser'(1:2,:);
Z(3:4,:)=laser'(3:4,:);

%initial guess solution
x=[1, 1];

%alignement loop
iterations=1;

for i=1:iterations
	H=zeros(2,2);
	B=zeros(2,1);
	J=zeros(size(2,1),2);
	K=zeros(size(2,1),1);
	for i=1:size(Z,2)
		J(1,1)=Z(3,i)*Z(4,i)*x(2)+Z(3,i); 			%jacobian
		J(1,2)=x(1)*Z(4,i)*Z(3,i); 				%jacobian
		K=Z(1,i)- ( Z(3,i)*x(1) - Z(3,i)*Z(4,i)*x(1)*x(2) ); 	%error
		H+=J'*J;						%hessian
		B+=(J'*K);						
	endfor
	dx=-H\B;
	x(1)=dx(1,1)+x(1);
	x(2)=dx(2,1)+x(2);
	
endfor

% SEE WHAT'S HAPPENED

%APPLIES THE CORRECTION
for i=1:size(laser,1)
	laser(i,3)=laser(i,3)+( Z(3,i)*x(1) - Z(3,i)*Z(4,i)*x(1)*x(2) );
endfor

%POLAR TO CARTESIAN
laserCartesian=zeros(size(laser));
for i=1:size(laser,1)
	[a,b]=polarToCartesian(laser(i,1),laser(i,2));
	laserCartesian(i,1)=a;
	laserCartesian(i,2)=b;
	[a,b]=polarToCartesian(laser(i,3),laser(i,4));
	laserCartesian(i,3)=a;
	laserCartesian(i,4)=b;
endfor

hold off
#axis([1 4 -1 3])
scatter(laserCartesian(:,3),laserCartesian(:,4),[],[1 0 0])
hold on
#axis([1 4 -1 3])
scatter(laserCartesian(:,1),laserCartesian(:,2),[],[0 1 0])
hold off
#axis([1 4 -1 3])

