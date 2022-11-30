close all;

view(3);
daspect([1 1 1]);
plot3([0,1],[0,0],[0,0],'Color',[1,0,0]);
hold on
plot3([0,0],[0,1],[0,0],'Color',[0,1,0]);
plot3([0,0],[0,0],[0,1],'Color',[0,0,1]);

T = homogeneousTrans(0, 0, pi/4,[1;1;1]);
O1 = T*[0;0;0;1];

v1 = T*[1;0;0;1]
v2 = T*[0;1;0;1]
v3 = T*[0;0;1;1]

plot3([O1(1),v1(1)],[O1(2),v1(2)],[O1(3),v1(3)],'Color',[1,0,0], 'LineStyle','--');
plot3([O1(1),v2(1)],[O1(2),v2(2)],[O1(3),v2(3)],'Color',[0,1,0],'LineStyle','--');
plot3([O1(1),v3(1)],[O1(2),v3(2)],[O1(3),v3(3)],'Color',[0,0,1], 'LineStyle','--');
