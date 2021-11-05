clear
close all
clc

l1=100;
l2=165;

epsilon = 5;
th1 = -90;
th2 = -70;

figure
for i=0:epsilon:180
    for j=0:epsilon:140      
        [x1,x2,y1,y2] = FCindir_RR(deg2rad(th1), deg2rad(th2), l1, l2);
        plot (x2, y2, 'b.');
        title('W-space')
        xlabel('x')
        ylabel('y')
        hold on
      
        axis([-(l1+l2) (l1+l2) -(l1+l2) (l1+l2)])        
       th2=th2+epsilon;
    end
    th1 = th1+epsilon;
    th2= -70;
end
                    