nL = 100;
nR = 1000;
A = 0.0011;
w = 0.3963;
wAng = 0.0069
deltaS = A*(nL+nR)/2;
deltaAlpha = (A*(nR-nL)/w);
cosDA = cos(deltaAlpha);
senDA = sin(deltaAlpha);
raiz = sqrt(2*(1-cosDA));
ThetaA = (106.317 * pi)/180; %angulo inicial en radianes
senDA1 = asin(senDA/raiz);

deltaX = - deltaS*raiz/deltaAlpha * sin(ThetaA - senDA1);
deltaY = deltaS*raiz/deltaAlpha * cos(ThetaA - senDA1);
deltaTheta = deltaAlpha*w/wAng;

%Imprimir valores
deltaS
deltaX
deltaY
deltaTheta
