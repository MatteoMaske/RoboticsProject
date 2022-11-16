% We draw a circle using the Jacobian Pseudoinverse control technique with
% redundnacy exploitation
clear all;
close all;
omega = 1;
DeltaT = 0.1;
laps = 2;
tMin = 0;
tMax = 2*pi/omega*laps;
global k0;
k0 = 20;
%xd = @(t) [0.4; 0.64*cos(omega*t); 0.64*sin(omega*t)];
xd = @(t) [0.2*cos(omega*t); 0.1; 0.2*sin(omega*t)];
TH0 = ur5Inverse(xd(0), eye(3,3));

K = 1*eye(3,3);


[Th]= invDiffKinematicControlSimRedundancy(@ur5Direct, @ur5Jac, xd, @qdot0, TH0(1,:), K, tMin, tMax, DeltaT);

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
for i = 2:max(size(Th)),
      pause;
    [pe,Re, handlesR] = ur5DirectDraw(Th(i,:), handlesR, false, scaleFactor);
    peA = [peA pe];
end
Samples = max(size(peA));
figure;
lim = scaleFactor*lim;           
axs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]); view(3); grid on;
xlabel('X', 'FontSize',12);ylabel('Y', 'FontSize',12);zlabel('Z', 'FontSize',12); hold on;
for i = 1:Samples-1,
   plot3([peA(1,i), peA(1,i+1)], [peA(2,i), peA(2,i+1)], [peA(3,i), peA(3,i+1)], 'Parent',axs);
end

figure;

[Th1]= invDiffKinematicControlSimRedundancy(@ur5Direct, @ur5Jac, xd, @qdotN, TH0(1,:), K, tMin, tMax, DeltaT);

lim = 1;
scaleFactor = 10;
limS = scaleFactor*lim;
axs1=axes('XLim',[-limS limS],'YLim',[-limS limS],'ZLim',[-limS limS]); view(3); grid on;
xlabel(['X x ', num2str(scaleFactor)], 'FontSize',12);
ylabel(['Y x ', num2str(scaleFactor)], 'FontSize',12);
zlabel(['Z x ', num2str(scaleFactor)], 'FontSize',12);
handles(1) = axs1;
[pe,Re, handlesR] = ur5DirectDraw(Th1(1,:), handles, true, scaleFactor);
peA1 = pe;
for i = 2:max(size(Th)),
      pause;
    [pe,Re, handlesR] = ur5DirectDraw(Th1(i,:), handlesR, false, scaleFactor);
    peA1 = [peA1 pe];
end
Samples = max(size(peA1));
figure;
lim = scaleFactor*lim;           
axs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]); view(3); grid on;
xlabel('X', 'FontSize',12);ylabel('Y', 'FontSize',12);zlabel('Z', 'FontSize',12); hold on;
for i = 1:Samples-1,       
   plot3([peA1(1,i), peA1(1,i+1)], [peA1(2,i), peA1(2,i+1)], [peA1(3,i), peA1(3,i+1)], 'Parent',axs);
end

figure
for i = 1:6,
    subplot(6,1,i)
    hold;
    plot(Th(:,i));
    plot(Th1(:,i));
end

                   
function [qd0] = qdot0(q)
global k0;
qd0 = -k0/6*(q/(2*pi));
end

function [qd0] = qdotN(q)
    qd0 = zeros(size(q));
end



