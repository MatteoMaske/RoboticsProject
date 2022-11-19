% Moves from an initial pose to
% a target frame representing an object

clear all;
close all;
Samples = 100;
T = linspace(0,1,Samples);
Th = [];
global xe0;
global xef;
global phie0;
global phief;
%Initial pose
xe0 = [0.3; 0.3; 0.1];
phie0 = [0; 0; 0];
xef = [0.5; 0.5; 0.5];
phief = [pi/4; pi/4; pi/4];



DeltaT = 0.01;  
TH0 = ur5Inverse(pd(0), eul2rotm(phid(0)'));

Kp = 10*eye(3,3);
Kphi = 0.0001*eye(3,3);

[Th]= invDiffKinematicControlSimComplete(@ur5Direct, @ur5Jac, @pd, @phid, TH0(1,:), Kp, Kphi, 0, 1, DeltaT);

 lim = 1;
scaleFactor = 10;
limS = scaleFactor*lim;
axs=axes('XLim',[-limS limS],'YLim',[-limS limS],'ZLim',[-limS limS]); view(3); grid on;
xlabel(['X x ', num2str(scaleFactor)], 'FontSize',12);
ylabel(['Y x ', num2str(scaleFactor)], 'FontSize',12);
zlabel(['Z x ', num2str(scaleFactor)], 'FontSize',12);
handles(1) = axs;
[pe,Re, handlesR] = ur5DirectDraw(Th(1,:), handles, true, scaleFactor);
Tt = [
    eul2rotm(phief') xef*scaleFactor;
    zeros(1,3) 1];
tt = hgtransform('Parent',axs, 'Matrix', Tt);
ht = triad('Parent',tt, 'linewidth', 3);
peA = pe;
phieA = rotm2eul(Re)';
for i = 2:max(size(Th)),
   %   pause;
    [pe,Re, handlesR] = ur5DirectDraw(Th(i,:), handlesR, false, scaleFactor);
    peA = [peA pe];
    phieA = [phieA rotm2eul(Re)'];
    pause
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
    if (t > 1),
        xd = xef;
    else
        xd = t*xef + (1-t)*xe0;
    end
end

function [phid] = phid(t)
global phief; global phie0;
    if (t > 1),
        phied = phief;
    else
        phid = t*phief + (1-t)*phie0;
    end
end

