A.a0=3; A.a2=3; A.a3=3;
Samples = 1000;
minTh1 = 0;
maxTh1 = pi/2;
minTh2 = 0;
maxTh2 =  pi/2;
minTh3 = 0; %-pi/2;
maxTh3 = pi/2; %pi/2;
TH.th3 = linspace(minTh3,maxTh3,Samples);
TH.th2 = linspace(minTh2,maxTh2,Samples);
TH.th1 = linspace(minTh1,maxTh1,Samples);
lim = 10;

pe = anthropomorphicTrans(TH, A, lim);
