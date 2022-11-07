colors = [  0, 0.4470, 0.7410;	   
          	0.8500, 0.3250, 0.0980;	         
          	0.9290, 0.6940, 0.1250;	       
          	0.4940, 0.1840, 0.5560;	      
          	0.4660, 0.6740, 0.1880;	    
          	0.3010, 0.7450, 0.9330;	  
          	0.6350, 0.0780, 0.1840;
            0.01,    0.4,    0.25;
            0.8,    0.1,    0.1];
load ('illcond_plot.mat', 'eDirect3pFit', 'eSun16', 'eShi19');

h = figure;
hold on;
%axis equal;
gSmooth = 30;
min = 50;
max = 150;
fs = 12;
lw = 2;
xl = 'No. of adjacent inliers';
yl = 'Err. of the sphere center estimation (m)';
xlim([min max]);
ylim([0 0.1]);
plot(min:max, smoothdata(eDirect3pFit(min:max), 'gaussian', gSmooth), 'Color', colors(1, :), 'LineWidth',lw);
plot(min:max, smoothdata(eSun16(min:max), 'gaussian', gSmooth), 'Color', colors(3, :), 'LineWidth',lw);
plot(min:max, smoothdata(eShi19(min:max), 'gaussian', gSmooth), 'Color', colors(9, :), 'LineWidth',lw);
plot(min:max, smoothdata(eDirect3pFit(min:max), 'gaussian', gSmooth), '--', 'Color', colors(1, :), 'LineWidth',lw);
legend('Proposed', 'LLSB', 'IMM');
xlabel(xl, 'FontSize', fs);
ylabel(yl, 'FontSize', fs);

outputFile= strcat('Matlab2_illcond_plot.pdf');
set(gca,'LooseInset',get(gca,'TightInset'));
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(outputFile,'-dpdf','-r0')
