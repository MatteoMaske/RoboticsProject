% This script is simply to test the forward kinematics of the ur5 robot
% Essentially we change one of the joints and we evaluate the effect on the
% animation.
clear all;
close all;
lim = 1;
scaleFactor = 10;
limS = scaleFactor*lim;
axs=axes('XLim',[-limS limS],'YLim',[-limS limS],'ZLim',[-limS limS]); view(3); grid on;
xlabel(['X x ', num2str(scaleFactor)], 'FontSize',12);
ylabel(['Y x ', num2str(scaleFactor)], 'FontSize',12);
zlabel(['Z x ', num2str(scaleFactor)], 'FontSize',12);
handles(1) = axs;
Th = [0, 0, 0, 0, 0, 0];
[pe,Re, handlesR] = ur5DirectDraw(Th, handles, true, scaleFactor);

for th = 0:0.1:2*pi,
    Th(3) = th;
    [pe,Re, handlesR] = ur5DirectDraw(Th, handlesR, false, scaleFactor);
     pause;
end
