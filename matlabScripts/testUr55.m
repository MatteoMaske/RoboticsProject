%This is hust a check that differential kineamtics actually works
clear all;
close all;
omega = 5*pi/2;
DeltaT = 0.1;
rounds = 40;
tMin = 0;
tMax = 2*pi/omega*rounds;
peA= [];
q = [0,0,0,0,0,0];
pe = ur5Direct( q );
vq = [4, 0, 0, 0, 0, 0];
for i = tMin:DeltaT:tMax,
    
    peA = [peA, pe];
    J = ur5Jac(q);
    pe = pe + J(1:3,1:6)*vq'*DeltaT;
    q = q + vq*DeltaT;
end
lim = 1;
scaleFactor = 10;
limS = scaleFactor*lim;
axs=axes('XLim',[-limS limS],'YLim',[-limS limS],'ZLim',[-limS limS]); view(3); grid on;
xlabel(['X x ', num2str(scaleFactor)], 'FontSize',12);
ylabel(['Y x ', num2str(scaleFactor)], 'FontSize',12);
zlabel(['Z x ', num2str(scaleFactor)], 'FontSize',12);
handles(1) = axs;

Samples = max(size(peA));
figure;
lim = scaleFactor*lim;           
naxs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]); view(3); grid on;
xlabel('X', 'FontSize',12);ylabel('Y', 'FontSize',12);zlabel('Z', 'FontSize',12); hold on;

peA = peA*scaleFactor;
for i = 1:Samples-1,
   plot3([peA(1,i), peA(1,i+1)], [peA(2,i), peA(2,i+1)], [peA(3,i), peA(3,i+1)], 'Parent',naxs);
end
    
