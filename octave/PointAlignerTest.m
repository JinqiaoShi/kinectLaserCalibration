# generate a random set od points
P = (rand(2,100)-.5)*100;
# create a guess transform
x_ideal=[20,30,pi/2]';
# get the homogeneous matrix
X_ideal=v2t(x_ideal);

#obtain the transformed points (by column)
Pt = transformPoints(inv(X_ideal),P);

#assemble the measurement matrix
# first two rows x a,d y of each "reference" point
# third and fourth rtow, x and y of each "current" point
# a column is  a measurement pi, pj;
Z=zeros(4,100);
Z(1:2,:)=P;
Z(3:4,:)=Pt;

x=[0,0,0]';
# now run the algorithm and get the chi2
[xes, chis,xnew] = pointAlignerLoop(x,Z,100)


