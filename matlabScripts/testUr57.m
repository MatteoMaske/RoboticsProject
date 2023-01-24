% Moves from an initial pose to
% a target frame representing an object

clear all;
close all;
Samples = 500;
global Tm
Tm = 1;
Tf =Tm;
T = linspace(0,Tf,Samples);
Th = [];
global xe0;
global xef;
global phie0;
global phief;
%Initial pose
xe0 = [0.3; 0.3; 0.1];
phie0 = [0; 0; 0];
xef = [0.4; 0.4; 0.5];
phief = [0; pi; pi]; %we assume ZYX convention but the vector is stored as X,Y,Z coordinates, so [0,0,pi] is a rotation of pi about Z axis

%TODO fix this there is the gimbal lock issue when pitch passes through
%pi/2!


DeltaT =Tf/Samples ;  
TH0 = ur5Inverse(pd(0), eul2rotm(phid(0)'));

Kp = 5*eye(3,3);
%Kphi = diag([0.1,0.1,0.5]);
Kphi = 0.1*eye(3,3);

%[Th]= invDiffKinematicControlSimComplete(@ur5Direct, @ur5Jac, @pd, @phid, TH0(1,:), Kp, Kphi, 0, Tf, DeltaT);
[Th]= invDiffKinematicControlSimCompleteAngleAxis(@ur5Direct, @ur5Jac, @pd, @phid, TH0(1,:), Kp, Kphi, 0, Tf, DeltaT);

 lim = 1;
scaleFactor = 10;
limS = scaleFactor*lim;
axs=axes('XLim',[-limS limS],'YLim',[-limS limS],'ZLim',[-limS limS]); view(3); grid on;
xlabel(['X x ', num2str(scaleFactor)], 'FontSize',12);
ylabel(['Y x ', num2str(scaleFactor)], 'FontSize',12);
zlabel(['Z x ', num2str(scaleFactor)], 'FontSize',12);
handles(1) = axs;
[pe,Re, handlesR] = ur5DirectDraw(Th(1,:), handles, true, scaleFactor);


Tt = [eul2rotmFDR(phief), xef*scaleFactor;
    zeros(1,3) 1];
tt = hgtransform('Parent',axs, 'Matrix', Tt);
ht = triad('Parent',tt, 'linewidth', 6);
peA = pe;
phieA = rotm2eul(Re)';



for i = 2:max(size(Th)),
   %   pause;
    [pe,Re, handlesR] = ur5DirectDraw(Th(i,:), handlesR, false, scaleFactor);
    peA = [peA pe];
    phieA = [phieA  rotm2eulFDR(Re)'];
    %pause(0.001)
    view(6.23, 8.31)
end
Samples = max(size(peA));
figure;
lim = scaleFactor*lim;           
axs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]); view(3); grid on;
xlabel('X', 'FontSize',12);ylabel('Y', 'FontSize',12);zlabel('Z', 'FontSize',12); hold on;
for i = 1:Samples-1,
   plot3([peA(1,i), peA(1,i+1)], [peA(2,i), peA(2,i+1)], [peA(3,i), peA(3,i+1)], 'Parent',axs);
end
    


function [xd] = pd(t)
global xef; global xe0;
global Tm;
    if (t > Tm),
        xd = xef;
    else
        xd = (t/Tm)*xef + (1-(t/Tm))*xe0;
    end
end

function [phid] = phid(t)
global phief; global phie0;
global Tm;
    if (t > Tm),
        phid = phief;
    else
        phid = (t/Tm)*phief + (1-(t/Tm))*phie0;
    end
end

