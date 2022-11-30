clear all;
close all;
%Define the axis
axs=axes('XLim',[-1.5 1.5],'YLim',[-1.5 1.5],'ZLim',[-1.5 1.5]);
view(3)
grid on;
% Create an empty transformation referred
% to the axis just created
t = hgtransform('Parent',axs);
%Create a cylinder
[x,y,z] = cylinder([1,1],100);

h = surface(x,y,z,'FaceColor','red');
%Associate the cylinder with the
%transformation object
set(h, 'Parent',t);
%Wait for a key to be pressed
pause
for angle = 0:0.5:360,
%create a rotation matrix
Rz = makehgtform('xrotate',deg2rad(angle));
%apply to the transformation object and to its children
set (t,'Matrix',Rz);
%Update the children objects
drawnow;
end