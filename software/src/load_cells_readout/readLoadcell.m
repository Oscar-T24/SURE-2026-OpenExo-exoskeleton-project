% This code is in the exact same format as the one for cubemars_CAN_arduino 
% Written on 6/12/2026 

% -----------------2 Load cells Testing ----------------------
% Type in readLoadcell("left") or readLoadcell("right") to plot voltage over time

function readLoadcell(sideToPlot) % choose between left and right load cell

% Remove current serial objects from memory
delete(serialportfind);

% Connect to Arduino
serialObj = serialport("COM3", 115200); % Adjust COM channel and baud rate as needed
configureTerminator(serialObj,"LF");
flush(serialObj);

pause(2); 

% Prepare to store Arduino data
serialObj.UserData = struct;
serialObj.UserData.Time = [];
serialObj.UserData.Value = [];
serialObj.UserData.SideToPlot = sideToPlot;


% Prepare plot labels according to the value we decide to plot
if sideToPlot == "left"
    plotTitle = "Left Loadcell Voltage vs Time";
    yaxis = "Left Loadcell Voltage (V)";
elseif sideToPlot == "right"
    plotTitle = "Right Loadcell Voltage vs Time";
    yaxis =  "Right Loadcell Voltage (V)";
else
    error("Please choose either left or right loadcell");
end

% Create figure

fig = figure;

h = animatedline("Linewidth", 2);
xlabel("Time (s)");
ylabel(yaxis);
title(plotTitle);
grid on;

serialObj.UserData.h = h;
serialObj.UserData.StartTime = tic;
fig.CloseRequestFcn = @(~,~)stopPlot(serialObj, fig);

% Read serial lines output from motor
configureCallback(serialObj, "terminator", @readLoadcellData);

end

function readLoadcellData(src, ~)

line = readline(src);
line = strtrim(line); % remove start and end blank spaces
words = split(line);

sideToPlot = src.UserData.SideToPlot;

if sideToPlot == "left"
    value = getValueOfSideToPlot(words, "Left");
elseif signalToPlot == "right"
    value = getValueOfSideToPlot(words, "Right:");
else
    return;
end

% Start Plotting
t = toc(src.UserData.StartTime);

addpoints(src.UserData.h, t, value);
drawnow limitrate;

end

function value = getValueOfSideToPlot(words, wordToFind)

index = find(words == wordToFind, 1);

% Can add if statements about cases where the word is not found and when it
% is the last word, but it should not happen

valueText = words(index + 3);
value = str2double(valueText);
end


function closeLoadcellPlot(serialObj, fig)

configureCallback(serialObj, "off");
clear serialObj;
delete(fig);

end
