close all
%Create some axis      
axs = axes;

%Set 3D view
view(3);

%Set aspect ratio
daspect([1 1 1]);

%Draw a first reference frame that coincides with the current axis
h = triad('Parent', axs);

%Now draw a second refernce frame obtained with a translation and a
%rotation about x
h1=triad('Parent',h,'Matrix',makehgtform('translate',[1,1,1],'xrotate',pi/4),'linewidth',3,'linestyle','--')

