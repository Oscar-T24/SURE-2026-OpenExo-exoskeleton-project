% Type readMotor("torque") or readMotor("velocity") to start plotting
% Type delete(serialportfind)

% ---------------------Set MIT Mode Parameters----------------------------

pos = 0.0;
vel = 0.0;
kd = 0.0;
trq = 0.0;

%----------------------START MOTOR TESTING-----------------------------

function readMotor(signalToPlot) % signalToPlot can be "torque" or "velocity"

% Send MIT Mode Parameters to Arduino Serial
writeline(serialObj, "pos");
writeline(serialObj, pos);
writeline(serialObj, "vel");
writeline(serialObj, vel);
writeline(serialObj, "kd");
writeline(serialObj, kd);
writeline(serialObj, "trq");
writeline(serialObj, trq);

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
serialObj.UserData.SignalToPlot = signalToPlot;


% Prepare plot labels according to the value we decide to plot
if signalToPlot == "torque"
    plotTitle = "Motor Torque vs Time";
    yaxis = "Torque (Nm)";
elseif signalToPlot == "velocity"
    plotTitle = "Motor Velocity vs Time";
    yaxis =  "Velocity (rad/s)";
else
    error("This code doesn't account for plotting values other than torque and velocity");
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
configureCallback(serialObj, "terminator", @readMotorData);

end

function readMotorData(src, ~)

line = readline(src);
line = strtrim(line); % remove start and end blank spaces
words = split(line);

signalToPlot = src.UserData.SignalToPlot;

if signalToPlot == "torque"
    value = getValueOfSignalToPlot(words, "trq:");
elseif signalToPlot == "velocity"
    value = getValueOfSignalToPlot(words, "vel:");
else
    return;
end

% Start Plotting
t = toc(src.UserData.StartTime);

addpoints(src.UserData.h, t, value);
drawnow limitrate;

end

function value = getValueOfSignalToPlot(words, wordToFind)

index = find(words == wordToFind, 1);

% Can add if statements about cases where the word is not found and when it
% is the last word, but it should not happen

valueText = words(index + 1);
value = str2double(valueText);
end

function closeMotorPlot(serialObj, fig)

configureCallback(serialObj, "off");
clear serialObj;
delete(fig);

end
