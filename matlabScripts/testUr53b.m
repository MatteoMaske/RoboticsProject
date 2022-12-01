% Moves from an initial pose to
% a target frame representing an object
Samples = 100;
T = linspace(0,1,Samples);
Th = [];

%Initial pose
%TH0 = [0, 0, 0, 0.01, 0.01, 0];
TH0 = [ -0.3223527113543909,-0.7805794638446351, -2.5675506591796875,-1.6347843609251917, -1.5715253988849085, -1.0017417112933558]
[xe0, Re] = ur5Direct(TH0);
phie0 = rotm2eul(Re)',

xef = [0.19; 0.2; -0.7 ];
phief = [0;0;0];
xe = @(t) t*xef+(1-t)*xe0;
phie = @(t) t*phief+(1-t)*phie0; 

for t = T,
   x = xe(t);
   phi=phie(t);
   [TH] = ur5Inverse(xe(t), eul2rotm(phie(t)'));
   Th= [Th; TH(1,:)],
end     

lim = 1;
scaleFactor = 10;
limS = scaleFactor*lim;
axs=axes('XLim',[-limS limS],'YLim',[-limS limS],'ZLim',[-limS limS]); view(3); grid on;

%DRAW THE TARGET FRAME

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
for i = 2:max(size(Th)),
     pause;
    [pe,Re, handlesR] = ur5DirectDraw(Th(i,:), handlesR, false, scaleFactor);
    peA = [peA pe];
end
figure;
lim = scaleFactor*lim;           
axs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]); view(3); grid on;
xlabel('X', 'FontSize',12);ylabel('Y', 'FontSize',12);zlabel('Z', 'FontSize',12); hold on;
for i = 1:Samples-1,
   plot3([peA(1,i), peA(1,i+1)], [peA(2,i), peA(2,i+1)], [peA(3,i), peA(3,i+1)], 'Parent',axs);
end
