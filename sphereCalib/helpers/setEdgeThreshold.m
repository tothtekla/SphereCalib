function edgeThreshold = setEdgeThreshold%(img)

edgeThreshold=0;
%img = rgb2gray(img);
%img_original = img;

fig = uifigure('Position',[100 100 350 275]);

% Create a UI axes
ax = uiaxes('Parent',fig,...
            'Units','pixels',...
            'Position', [104, 123, 300, 201]);   

% Create a push button
btn = uibutton(fig,'push',...
               'Position',[420, 218, 100, 22],...
               'ButtonPushedFcn', @(btn,event) plotButtonPushed(btn,ax));

% Create figure window and components
cg = uigauge(fig,'Position',[100 100 120 120]);

sld = uislider(fig,...
    'Position',[100 75 120 3],...
    'ValueChangedFcn',@(sld,event) updateGauge(sld,edgeThreshold));
pause(0);
disp(edgeThreshold);
end

% Create ValueChangedFcn callback
function updateGauge(sld,edgeThreshold)
edgeThreshold= sld.Value;
end

% Create the function for the ButtonPushedFcn callback
function plotButtonPushed(btn,ax)
        x = linspace(0,2*pi,100);
        y = sin(x);
        plot(ax,x,y)
end