% Remove current serial objects from memory
delete(serialportfind);

% Connect to Arduino
serialObj = serialport("COM7", 115200); % Adjust COM channel and baud rate as needed
configureTerminator(serialObj,"LF");
flush(serialObj);

pause(2); % Arduino reset delay?

% Place ankle at neutral position and zero it
writeline(serialObj,"z");
pause(0.2);

% Prepare to store Arduino data
serialObj.UserData = struct("Data",[],"Count",1);

% Create plot and set parameters
fig = figure;

h = animatedline("LineWidth",1.5);
xlabel("Time (s)");
ylabel("Relative angle (deg)");
title("AMT20 Encoder Relative Angle vs Time");
grid on;

serialObj.UserData.h = h; % Use plot line as the place where data is stored
serialObj.UserData.StartTime = tic;

fig.CloseRequestFcn = @(~,~)stopPlot(serialObj, fig);

configureCallback(serialObj, "terminator", @readEncoderData);

%Start encoder test
writeline(serialObj,"y");

function readEncoderData(src, ~)

    line = readline(src);
    angleDeg = str2double(strtrim(line));

    if isnan(angleDeg)
        return;
    end
    
    t = toc(src.UserData.StartTime);
    
    addpoints(src.UserData.h,t,angleDeg);
    drawnow limitrate;

end

% These may not work -- instead type writeline(serialObj, "n") in Matlab
% Command window

% Type "stopTest(serialObj) in Matlab comand window 
function stopTest(serialObj)

    try
        writeline(serialObj, "n");   % stop Arduino streaming
        pause(0.1);
    catch
    end
    
    configureCallback(serialObj, "off");

    disp("Encoder test stopped");

end

function closePlot(serialObj, fig)

    stopStream(serialObj);

    delete(fig);
end
