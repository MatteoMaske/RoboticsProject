% Moves from an initial pose to
% a target frame representing an object
Samples = 100;
T = linspace(0,1,Samples);
Th = [];

%Initial pose
TH0 = [0, 0, 0, 0.01, 0.01, 0]; %Initial pose
[xe0, Re] = ur5Direct(TH0); %Forward kinematics that returns the end-effector position and orientation
phie0 = rotm2eul(Re)'; %Converts the orientation matrix to Euler angles

xef = [0.5; 0.5; 0]; %Target position
phief = [pi; pi/4; 3*pi/4]; %Target orientation
xe = @(t) t*xef+(1-t)*xe0; %Linear interpolation of the position
phie = @(t) t*phief+(1-t)*phie0;    %Linear interpolation of the orientation

%Inverse kinematics for each sample of the trajectory and store the joint angles in Th matrix
for t = T, 
   x = xe(t);
   phi=phie(t);
   [TH] = ur5Inverse(xe(t), eul2rotm(phie(t)'));
   Th= [Th; TH(1,:)];   
end

%Plot the joint angles

lim = 1;
scaleFactor = 10;
limS = scaleFactor*lim;
axs=axes('XLim',[-limS limS],'YLim',[-limS limS],'ZLim',[-limS limS]); view(3); grid on;

%DRAW THE TARGET FRAME

xlabel(['X x ', num2str(scaleFactor)], 'FontSize',12); 
ylabel(['Y x ', num2str(scaleFactor)], 'FontSize',12);
zlabel(['Z x ', num2str(scaleFactor)], 'FontSize',12);
handles(1) = axs; %Store the handle of the axes in the handles vector for later use

[pe,Re, handlesR] = ur5DirectDraw(Th(1,:), handles, true, scaleFactor); %Draw the initial pose of the robot
Tt = [
    eul2rotm(phief') xef*scaleFactor;
    zeros(1,3) 1]; %Transformation matrix of the target frame
tt = hgtransform('Parent',axs, 'Matrix', Tt); %Create a transformation object
ht = triad('Parent',tt, 'linewidth', 3); %Draw the target frame
             
peA = pe; %Store the end-effector position for later use
for i = 2:max(size(Th)), %Draw the robot for each sample of the trajectory
     pause;
    [pe,Re, handlesR] = ur5DirectDraw(Th(i,:), handlesR, false, scaleFactor);
    peA = [peA pe];
end
figure; %Plot the end-effector position

lim = scaleFactor*lim; %Scale the limits of the plot      
axs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]); view(3); grid on; %Set the axes limits and view
xlabel('X', 'FontSize',12);ylabel('Y', 'FontSize',12);zlabel('Z', 'FontSize',12); hold on; %Set the axes labels and hold the plot
for i = 1:Samples-1, %Draw the trajectory of the end-effector
   plot3([peA(1,i), peA(1,i+1)], [peA(2,i), peA(2,i+1)], [peA(3,i), peA(3,i+1)], 'Parent',axs);
end
