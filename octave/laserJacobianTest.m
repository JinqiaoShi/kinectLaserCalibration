laser=load("laserdataset.m");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%measurement matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Z=zeros(4,size(laserCartesian,1));
Z(1:2,:)=laserCartesian'(1:2,:);
Z(3:4,:)=laserCartesian'(3:4,:);



%COMPUTES ERROR BEFORE OPTIMIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fullerror=0;
for i=1:size(Z,2)
	ro2=Z(1,i)^2+ Z(2,i)^2;
	ro2prime=Z(3,i)^2+ Z(4,i)^2;
	fullerror+= sqrt(ro2)-sqrt(ro2prime);
endfor
fullerror



%initial guess 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k=[0;0];




%alignement loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

iterations=100;
for i=1:iterations
	kk=i;
	H=zeros(2,2);
	B=zeros(2,1);
	J=zeros(1,2);
	K=0;
	for i=1:size(Z,2)
		Py=Z(4,i);
		Px=Z(3,i);
		
		roFake=sqrt(Px^2+Py^2);
		alfaFake=atan(Py/Px);
		
		J(1,1)= -roFake;
		J(1,2)= -roFake*alfaFake^2;
		
		ro2prime=roFake*k(1)+roFake*k(2)*alfaFake^2;
		ro2=sqrt(Z(1,i)^2+Z(2,i)^2);
		K= ro2-( sqrt(Z(3,i)^2+Z(4,i)^2) + ro2prime);
		
		H+=J'*J;
		B+=(J'*K);						
	endfor
	dx=-H\B;
	k(1)+=dx(1);
	k(2)+=dx(2);
	
	%{############### IGNORE THIS ###################
	if(dx(1)^2+dx(2)^2<0.000001)
		printf("Stopped at %d iteration, no relevant increment (%f,%f)\n",kk,dx(1),dx(2))
		break;
	endif
	%}############### IGNORE THIS ###################
endfor

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SEE WHAT'S HAPPENED
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%APPLIES THE CORRECTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:size(Z,2)
	Px=Z(3,i);
	Py=Z(4,i);
	roFake=sqrt(Px^2+Py^2);
	alfaFake=atan(Py/Px);
	ro_prime=roFake*k(1)+roFake*k(2)*alfaFake^2;
	Z(3,i)+=(ro_prime)*cos(alfaFake);
	Z(4,i)+=(ro_prime)*sin(alfaFake);
endfor

%COMPUTES ERROR AFTER OPTIMIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fullerror=0;
for i=1:size(Z,2)
	ro2=(Z(1,i)^2+ Z(2,i)^2);
	ro2prime=(Z(3,i)^2+ Z(4,i)^2);
	fullerror+= sqrt(ro2)-sqrt(ro2prime);
endfor
fullerror


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PLOTTING DATA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hold off
#axis([1 4 -1 3])
scatter(Z(3,:),Z(4,:),[],[1 0 0])
hold on
#axis([1 4 -1 3])
scatter(Z(1,:),Z(2,:),[],[0 1 0])
hold off
#axis([1 4 -1 3])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
