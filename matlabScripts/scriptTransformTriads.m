clear all;
close all;
%Define the axis
axs=axes('XLim',[-2.5 1.5],'YLim',[-2.5 2.5],'ZLim',[-2.5 2.5]);
view(3)
grid on;
% Create an empty transformation referred
% to the axis just created
t = hgtransform('Parent',axs);
h = triad('Parent', t);
%Now draw a second refernce frame obtained with a translation and a
%rotation about x
h1=triad('Parent',h,'Matrix',makehgtform('translate',[1,1,1],'xrotate',pi/4))
plot3([0,1],[0,1],[0,1],'Color',[1,0,0], 'Parent',h, 'linewidth',3, 'linestyle','--');
%Wait for a key to be pressed
pause
for angle = 0:0.5:360,
%create a rotation matrix
Rz = makehgtform('zrotate',deg2rad(angle));
%apply to the transformation object and to its children
set (t,'Matrix',Rz);
%Update the children objects
drawnow;
end