%Let us draw a circle using differential kinematics    
clear all;

omega = 1;
DeltaT = 0.1;
laps = 10  ;
tMin = 0;
tMax = 2*pi/omega*laps;
%ve = @(t) [0;-0.2*omega*sin(omega*t); 0.2*omega*cos(omega*t)];
ve = @(t) [-0.2*omega*sin(omega*t); 0; 0.2*omega*cos(omega*t)];
omegae = @(t) [0;0; 0];
[TH0] = ur5Inverse([0.2;0.2;0], eye(3,3));
%TH0 = [0, 0,0, 0, 0, 0]; 
[Th]= invDiffKinematic( @ur5Jac, ve, omegae,TH0(1,:), tMin, tMax, DeltaT);

 lim = 1;
scaleFactor = 10;
limS = scaleFactor*lim;
axs=axes('XLim',[-limS limS],'YLim',[-limS limS],'ZLim',[-limS limS]); view(3); grid on;
xlabel(['X x ', num2str(scaleFactor)], 'FontSize',12);
ylabel(['Y x ', num2str(scaleFactor)], 'FontSize',12);
zlabel(['Z x ', num2str(scaleFactor)], 'FontSize',12);
handles(1) = axs;
[pe,Re, handlesR] = ur5DirectDraw(Th(1,:), handles, true, scaleFactor);
peA = pe;
phieA = rotm2eul(Re)';
for i = 2:max(size(Th)),
      pause;
    [pe,Re, handlesR] = ur5DirectDraw(Th(i,:), handlesR, false, scaleFactor);
    peA = [peA pe];
    phieA = [phieA  rotm2eul(Re)'];
end
Samples = max(size(peA));
figure;
lim = scaleFactor*lim;           
axs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]); view(3); grid on;
xlabel('X', 'FontSize',12);ylabel('Y', 'FontSize',12);zlabel('Z', 'FontSize',12); hold on;
for i = 1:Samples-1,
   plot3([peA(1,i), peA(1,i+1)], [peA(2,i), peA(2,i+1)], [peA(3,i), peA(3,i+1)], 'Parent',axs);
end
    
