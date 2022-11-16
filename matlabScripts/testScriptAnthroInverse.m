A.a0=3; A.a2=3; A.a3=3;
Samples = 100;
T = linspace(0,1,Samples);
Th.th1 = [];
Th.th2 = [];
Th.th3 = [];

for t = T,
   x = [1.8*cos(2*pi*t), 3, 1.8*sin(2*pi*t)]';
   [th1, th2, th3] = anthropomorphicInverse(x,A);
    Th.th1 = [Th.th1, th1(2)];
    Th.th2 = [Th.th2, th2(2)];
    Th.th3 = [Th.th3, th3(2)];  
end
lim = 10;
pe = anthropomorphicTrans(Th, A, lim); pe(3,:)=pe(3,:)+A.a0;
figure;
axs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]); view(3); grid on;
xlabel('X', 'FontSize',12);ylabel('Y', 'FontSize',12);zlabel('Z', 'FontSize',12); hold on;
for i = 1:Samples-1,
   plot3([pe(1,i), pe(1,i+1)], [pe(2,i), pe(2,i+1)], [pe(3,i), pe(3,i+1)], 'Parent',axs);
end