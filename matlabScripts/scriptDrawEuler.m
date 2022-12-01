heclose all;

view(3);
daspect([1 1 1]);
plot3([0,1],[0,0],[0,0],'Color',[1,0,0]);
hold on
plot3([0,0],[0,1],[0,0],'Color',[0,1,0]);
plot3([0,0],[0,0],[0,1],'Color',[0,0,1]);

R = eulerRot(pi/6, pi/6, 0);

v1 = R*[1;0;0]
v2 = R*[0;1;0]
v3 = R*[0;0;1]

plot3([0,v1(1)],[0,v1(2)],[0,v1(3)],'Color',[1,0,0], 'LineStyle','--');
plot3([0,v2(1)],[0,v2(2)],[0,v2(3)],'Color',[0,1,0],'LineStyle','--');
plot3([0,v3(1)],[0,v3(2)],[0,v3(3)],'Color',[0,0,1], 'LineStyle','--');
