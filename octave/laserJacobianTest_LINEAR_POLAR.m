laser=load("laserdataset.m");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%measurement matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Z=zeros(4,size(laser,1));
Z(1:2,:)=laser'(1:2,:);
Z(3:4,:)=laser'(3:4,:);



%COMPUTES ERROR BEFORE OPTIMIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fullerror=0;
for i=1:size(laser,1)
	fullerror+=(laser(i,1)-laser(i,3))^2;
endfor
fullerror



%initial guess solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x=[0,0];




%alignement loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
iterations=1;
plotUtil=zeros(iterations,3);
plotUtil(:,1)=1:iterations;
for i=1:iterations
	kk=i;
	H=zeros(2,2);
	B=zeros(2,1);
	J=zeros(1,2);
	K=zeros(size(2,1),1);
	for i=1:size(Z,2)
		alfa=Z(4,i);
		scan=Z(3,i);
		J(1,1)=-scan;
		J(1,2)=-scan*alfa^2;
		K=  Z(1,i)-(scan + scan*x(1)+scan*(alfa^2)*x(2) ) ;
		H+=J'*J;
		B+=(J'*K);						
	endfor
	dx=-H\B;
	x(1)+=dx(1);
	x(2)+=dx(2);
	
	%{
	if(dx(1)^2+dx(2)^2<0.0001)
		printf("Stopped at %d iteration, no relevant increment (%f,%f)\n",kk,dx(1),dx(2))
		break;
	endif
	%}
endfor

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SEE WHAT'S HAPPENED
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%APPLIES THE CORRECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:size(laser,1)
	alfa=laser(i,4);
	scan=laser(i,3);
	laser(i,3)=scan+scan*x(1)+scan*(alfa^2)*x(2);
endfor


fullerror=0;
for i=1:size(laser,1)
	fullerror+=(laser(i,1)-laser(i,3))^2;
endfor
fullerror


%POLAR TO CARTESIAN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
laserCartesian=zeros(size(laser));
for i=1:size(laser,1)
	[a,b]=polarToCartesian(laser(i,1),laser(i,2));
	laserCartesian(i,1)=a;
	laserCartesian(i,2)=b;
	[a,b]=polarToCartesian(laser(i,3),laser(i,4));
	laserCartesian(i,3)=a;
	laserCartesian(i,4)=b;
endfor
















%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold off
#axis([1 4 -1 3])
scatter(laserCartesian(:,3),laserCartesian(:,4),[],[1 0 0])
hold on
#axis([1 4 -1 3])
scatter(laserCartesian(:,1),laserCartesian(:,2),[],[0 1 0])
hold off
#axis([1 4 -1 3])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
