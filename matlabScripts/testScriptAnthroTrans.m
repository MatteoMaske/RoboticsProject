A.a0=3; A.a2=3; A.a3=3;
Samples = 100;
minTh1 = 0;
maxTh1 = 2*pi;
minTh2 = 0;
maxTh2 =  0;
minTh3 = 0; %-pi/2;
maxTh3 = 0; %pi/2;
TH.th3 = linspace(minTh3,maxTh3,Samples);
TH.th2 = linspace(minTh2,maxTh2,Samples);
TH.th1 = linspace(minTh1,maxTh1,Samples);
lim = 10;
pe = anthropomorphicTrans(TH, A, lim); 
figure;
axs=axes('XLim',[-lim lim],'YLim',[-lim lim],'ZLim',[-lim lim]); view(3); grid on;
xlabel('X', 'FontSize',12);
ylabel('Y', 'FontSize',12);
zlabel('Z', 'FontSize',12);
hold on;
for i = 1:Samples-1,
   plot3([pe(1,i), pe(1,i+1)], [pe(2,i), pe(2,i+1)], [pe(3,i), pe(3,i+1)], 'Parent',axs);
end