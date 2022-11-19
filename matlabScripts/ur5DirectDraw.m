%Direct Kinematics of the + draw UR5
%Th: six joint angles
%pe: cartesian position of the end effector
%Re: Rotation matrix of the end effecto
%handles: handles to the differetn transofmrations. THe first element links to the figure 
%firstTime
function [pe,Re, handlesR] = ur5DirectDraw(Th, handles, firstTime, scaleFactor)
  %Vector of the a distance (expressed in metres)
  A = [0, -0.425, -0.3922, 0, 0, 0]*scaleFactor;
  %Vector of the D distance (expressed in metres)
  D = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]*scaleFactor;
  
  %Computation of the transition matrices
  T10 = [
	 cos(Th(1)), -sin(Th(1)), 0, 0;
	 sin(Th(1)), cos(Th(1)), 0, 0;
	 0, 0, 1, D(1);
	 0, 0, 0, 1
  ];
  T21 = [
	 cos(Th(2)), -sin(Th(2)), 0, 0;
	 0, 0, -1, 0;
	 sin(Th(2)), cos(Th(2)), 0, 0;
	 0, 0, 0, 1
  ];
  T32 = [
	 cos(Th(3)), -sin(Th(3)), 0, A(2);
	 sin(Th(3)), cos(Th(3)), 0, 0;
	 0, 0, 1, 0;
	 0, 0, 0, 1
  ];
  T43 = [
	 cos(Th(4)), -sin(Th(4)), 0, A(3);
	 sin(Th(4)), cos(Th(4)), 0, 0;
	 0, 0, 1, D(4);
	 0, 0, 0, 1
  ];
  T54 = [
	 cos(Th(5)), -sin(Th(5)), 0, 0;
	 0, 0, -1, -D(5);
	 sin(Th(5)), cos(Th(5)), 0, 0;
	 0, 0, 0, 1
  ];
  T65 = [
	 cos(Th(6)), -sin(Th(6)), 0, 0;
	 0, 0, 1, D(6);
	 -sin(Th(6)), -cos(Th(6)), 0, 0;
	 0, 0, 0, 1
  ];
  T60 = T10*T21*T32*T43*T54*T65;
  
  %Computation of the end effector's position
  pe = T60(1:3,4);
  Re = T60(1:3, 1:3);
  axs = handles(1);
  % If first time draw, else update
  if firstTime,
      %Draw first frame (basis)
      h0 = triad('Parent', axs,'linewidth',3);
  
      %Transform to the end of the links and draw the frames and the links
      t1 = hgtransform('Parent',h0, 'Matrix', T10);
      h1 = triad('Parent',t1, 'linewidth', 3);
      O10 = T10(1:3,4);
      O00 = [0;0;0];
      plot3([O00(1), O10(1)], [O00(2), O10(2)], [O00(3), O10(3)], 'Parent',h0,'linestyle','--');
    
      
      t2 = hgtransform('Parent',t1, 'Matrix', T21);
      h2 = triad('Parent',t2,'linewidth', 3);  
  
      O32 = T32(1:3,4);
  
      t3 = hgtransform('Parent',t2, 'Matrix', T32);
      h3 = triad('Parent',t3,'linewidth', 3);  
      plot3([O32(1), 0], [O32(2), 0], [O32(3), 0], 'Parent',t2,'linestyle','--');
  
  
      t4 = hgtransform('Parent',t3, 'Matrix', T43);
      h4 = triad('Parent',t4,'linewidth', 3);  
      Op43 = [A(3), 0, 0];
      O43 = T43(1:3,4);
      
      plot3([Op43(1), 0], [Op43(2), 0], [Op43(3), 0], 'Parent',t3, 'linestyle','--')
      plot3([Op43(1), O43(1)], [Op43(2), O43(2)], [Op43(3), O43(3)], 'Parent',t3,'linestyle','--')
  
  
      t5 = hgtransform('Parent',t4, 'Matrix', T54);
      h5 = triad('Parent',t4,'linewidth', 3);  
      O54 = T54(1:3, 4);
      plot3([O54(1), 0], [O54(2), 0], [O54(3), 0], 'Parent',t4,'linestyle','--')
      
      t6 = hgtransform('Parent',t5, 'Matrix', T65);
      h6 = triad('Parent',t6,'linewidth', 3); 
      O65 = T65(1:3,4);
      plot3([O65(1), 0], [O65(2), 0], [O65(3), 0], 'Parent',t5,'linestyle','--')
  else
      t1 = handles(2);
      t2 = handles(3);
      t3 = handles(4);
      t4 = handles(5);
      t5 = handles(6);
      t6 = handles(7);
      
      set(t1, 'Matrix', T10);
      set(t2, 'Matrix', T21);
      set(t3, 'Matrix', T32),
      set(t4, 'Matrix', T43);
      set(t5, 'Matrix', T54);
      set(t6, 'Matrix', T65);
      drawnow;
  end
  handlesR(1) = axs;
  handlesR(2) = t1;
  handlesR(3) = t2;
  handlesR(4) = t3;
  handlesR(5) = t4;
  handlesR(6) = t5;
  handlesR(7) = t6;
end
