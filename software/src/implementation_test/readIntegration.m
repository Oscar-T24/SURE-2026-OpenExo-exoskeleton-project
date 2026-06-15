% ------------------------------ NOT TESTED YET --------------------------------------

% Connect to Teensy

delete(serialportfind);

serialObj = serialport("COM4", 115200); % Adjust COM channel and baud rate as needed
configureTerminator(serialObj,"CR/LF");
flush(serialObj);

pause(2);

% Prepare to store data

serialObj.UserData = struct;
serialObj.UserData.h = struct();
serialObj.UserData.fig = struct();
serialObj.UserData.t0 = tic;

configureCallback(serialObj, "terminator", @readIntegration);

% Define callback function

function readIntegration(src, ~)

line = readline(src);
line = strtrim(line);

if ~contains(line, ":") && ~contains(line, ",") % Condition to ignore all status messages
    return
end
    
t = toc(src.UserData.t0);

arr = split(line, ","); % Splits string into array of the format ["Right encoder :1234", "Left encoder: 567"]
i = 1; % starting index

ylabels = string(0);
val = [];

for element = arr
    element = strtrim(element);

    if element == ""
        continue
    end

    ab = split(element, ":");
    y = strtrim(ab(1));
    value = str2double(strtrim(ab(2)));

    if isnan(value)
        continue
    end

    ylabels(end+1) = y;
    val(end+1) = value;

    % Create the correct amount of plots once
    
    field = matlab.lang.makeValidName(y);
    
    if ~isfield(src.UserData.h, field)
    
        src.UserData.fig.(field) = figure("Name", y);
        src.UserData.h.(field) = animatedline("Linewidth", 1.5);
    
        xlabel("Time (s)");
        ylabel(y);
        title(y + " Plot Over Time");
        grid on;
        hold on;
    end

    % Update plot in real time

    addpoints(src.UserData.h.(field), t, value);

end

drawnow limitrate;

end


