nL = [200 500 2000 500];
nR = [400 1000 1000 2000];
DeltaAlpha = [30.375 80.19 -162.81 242.3925];
DeltaAlphaRad = [30.375 80.19 -162.81 242.3925] * pi/180;
A=0.0011;
w = mean(A*(nR - nL)./DeltaAlphaRad);
wGrad = mean(A*(nR - nL)./DeltaAlpha);