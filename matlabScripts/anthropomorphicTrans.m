%Draws an athropomorphic arm
% th1, th2, th3: joint variables
%A : lengths of the links
%lim: limits of the axis
function [pe] = anthropomorphicTrans(Th, A, lim )
figure;
a0 = A.a0; a2 = A.a2, a3 = A.a3;
th1=Th.th1; th2=Th.th2; th3 = Th.th3;

%Define the axis
axs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]);
view(3); 
grid on;
xlabel('X', 'FontSize',12);
ylabel('Y', 'FontSize',12);
zlabel('Z', 'FontSize',12);

A0b = [eye(3,3), [0;0;a0]; [0 0 0] 1];
hb = triad('Parent', axs,'linewidth',3); 
h0 = triad('Parent',hb,'Matrix', A0b,'linewidth', 3);
plot3([0, 0], [0, 0], [0, a0], 'Parent',hb,'linestyle','--');

%Transformation matrix from frame 0 to frame 1
A10 = [[cos(th1(1)) 0 sin(th1(1)); sin(th1(1)) 0 -cos(th1(1)); 0 1 0], [0;0;0];
        [0, 0, 0], 1];
t10 = hgtransform('Parent',h0, 'Matrix', A10);
h1 = triad('Parent', t10, 'linewidth', 3);


A21 = [[cos(th2(1)) -sin(th2(1)) 0; sin(th2(1)) cos(th2(1)) 0; 0 0 1], [a2*cos(th2(1));a2*sin(th2(1));0]; 
    [0 0 0] 1];
t21 = hgtransform('Parent',t10, 'Matrix', A21);
h2 = triad('Parent', t21, 'linewidth', 3);
plot3([0, -a0], [0, 0], [0, 0], 'Parent',t21,'linestyle','--');

A32 =  [[cos(th3(1)) -sin(th3(1)) 0; sin(th3(1)) cos(th3(1)) 0; 0 0 1], [a3*cos(th3(1));a3*sin(th3(1));0]; 
    [0 0 0] 1];
t32 = hgtransform('Parent',t21, 'Matrix', A32);
h3 = triad('Parent', t32, 'linewidth', 3);
plot3([0, -a0], [0, 0], [0, 0], 'Parent',t32,'linestyle','--');

k = max(size(th3));
pe = [];

for i = 1:k,
        A10 = [[cos(th1(i)) 0 sin(th1(i)); sin(th1(i)) 0 -cos(th1(i)); 0 1 0], [0;0;0];
            [0, 0, 0], 1];
        A21 = [[cos(th2(i)) -sin(th2(i)) 0; sin(th2(i)) cos(th2(i)) 0; 0 0 1], [a2*cos(th2(i));a2*sin(th2(i));0]; 
            [0 0 0] 1]; 
        A32 =  [[cos(th3(i)) -sin(th3(i)) 0; sin(th3(i)) cos(th3(i)) 0; 0 0 1], [a3*cos(th3(i));a3*sin(th3(i));0]; 
            [0 0 0] 1]
        set(t10, 'Matrix', A10);
        set(t21, 'Matrix', A21);
        set(t32, 'Matrix', A32);
        A30 = A10*A21*A32;
        Oe = A30*[0;0;0;1];
        pe = [pe Oe(1:3)];
        drawnow;
        pause;
end
end