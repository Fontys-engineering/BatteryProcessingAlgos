clear
clc

reflection_coefs = [1/3 0.1 8/10 0.25 1];
delayLengths = [100 100 100 100 100];
num_impedances = length(reflection_coefs);
stepTime = 2.5e-12;
stopTime = 10e-9;
inputMag = 3;

simIn = Simulink.SimulationInput("model");
simIn = simIn.setBlockParameter("model/DUT","NUM_IMPEDANCES",string(num_impedances));
formatString = "[";
for i = 1:num_impedances
    formatString = formatString + "%d ";
end
formatString = formatString + "]";
simIn = simIn.setBlockParameter("model/DUT","REFLECT_COEF",sprintf(formatString, reflection_coefs));
simIn = simIn.setBlockParameter("model/DUT","DELAY_LENGTH",sprintf(formatString, delayLengths));
simIn = simIn.setBlockParameter("model/inputGain","Gain",string(inputMag));
simIn = simIn.setModelParameter("FixedStep",string(stepTime));
simIn = simIn.setModelParameter("StopTime",string(stopTime));
out = sim(simIn);

% stim = out.yout{1}.Values;
reflectionData = squeeze(getdatasamples(out.yout{2}.Values,1:stopTime/stepTime+1))';
time = out.tout';

% plot(time*1e9, reflectionData)
% xlabel("Time (ns)")
% ylabel("Amplitude")
% title("Impulse response")

% Normalize reflection values
reflectionData = reflectionData/inputMag;

calcReflectionCoefs = [];
calcPropDelay = [];

% TODO: automate path generation
% paths = {[],[],[1 0 0 0 1],[1 1 0 0 0 1 1; 1 1 0 1 0 0 1; 1 0 0 1 0 1 1; 1 0 0 0 0 0 1]};
% for i=1:num_impedances
%     [calcReflectionCoefs(i), calcPropDelay(i), reflectionData] = calcStage(reflectionData, calcReflectionCoefs, calcPropDelay, paths{i});
% end

completedPaths = {};
for i=1:num_impedances
    [paths, completedPaths] = calcInternalPaths(calcPropDelay, i*200, completedPaths);
    [calcReflectionCoefs(i), calcPropDelay(i), reflectionData] = calcStage(reflectionData, calcReflectionCoefs, calcPropDelay, paths);
end

disp("Calculated reflection coefficients:");disp(calcReflectionCoefs)
disp("Actual reflection coefficients:");disp(reflection_coefs)
disp("Calculated propagation delay:");disp(calcPropDelay)
disp("Actual propagation delay:");disp(delayLengths)

function [value, delay, data] = calcStage(reflectionData, calcReflectionCoefs, calcPropDelay, paths)
    % Calculate all relevant signal paths
    delays = [];
    values = [];
    for i=1:numel(paths)
        state = 1;
        direction = true;
        value = 1;
        delay = calcPropDelay(1);
        for j=1:numel(paths{i})
            if paths{i}(j) == 1
                if direction
                    value = value * (1+calcReflectionCoefs(state));
                    delay = delay + calcPropDelay(state+1);
                    state = state + 1;
                else
                    value = value * (1-calcReflectionCoefs(state));
                    delay = delay + calcPropDelay(state);
                    state = state - 1;
                end
            else
                if direction
                    value = value * calcReflectionCoefs(state);
                    delay = delay + calcPropDelay(state);
                    state = state - 1;
                else
                    value = value * -calcReflectionCoefs(state);
                    delay = delay + calcPropDelay(state+1);
                    state = state + 1;
                end
                direction = ~direction;
            end
            if state == 0
                break
            end
        end
        delays = cat(2,delays,delay);
        values = cat(2,values,value);
    end

    % Subtract value from reflection data
    for i=1:numel(delays)
        reflectionData(delays(i)) = reflectionData(delays(i)) - values(i);
    end
    reflectionData(reflectionData < 1e-12) = 0;

    % Find first non-zero value
    idx = find(reflectionData,1);

    value = reflectionData(idx)/prod(1-calcReflectionCoefs.^2);
    delay = idx/2 - sum(calcPropDelay);
    reflectionData(idx) = reflectionData(idx)-value;
    data = reflectionData;
end

% Calculate all possible internal paths of at most maxDelay using numel(propDelay) stages.
% Returns every legal path defined as:
%  * Ending at stage 0
%  * Never reaching a stage < 0 or > numel(propDelay)
%  * At most maxDelay propagation delay
% Paths are returned ass cell array
% For each path, a 0 is a reflection and a 1 is a transmission
function [paths, completedPaths] = calcInternalPaths(propDelay, maxDelay, completedPaths)
    % Handle special case
    if numel(propDelay) == 0
        paths = {};
        return
    end

    % Calculate all paths to check
    minDelay = min(propDelay);
    maxTransitions = floorDiv(maxDelay,minDelay);
    pathsString = dec2bin(0:2^maxTransitions-1);
    pathsLogical = boolean(zeros(size(pathsString)));
    numStages = numel(propDelay);
    propDelay(end+1) = 0; % Avoid out of bounds indexing
    for i=1:size(pathsString,1)
        for j=1:maxTransitions
            pathsLogical(i,j) = (pathsString(i,j) == '1');
        end
    end

    % Remove paths that don't end at start
    validPaths = {};
    for i=1:size(pathsLogical,1)
        stage = 1;
        direction = true;
        delay = propDelay(1);
        validPath = true;
        for j=1:maxTransitions
            if pathsLogical(i,j)
                if direction
                    delay = delay + propDelay(stage+1);
                    stage = stage + 1;
                else
                    delay = delay + propDelay(stage);
                    stage = stage - 1;
                end
            else
                if direction
                    delay = delay + propDelay(stage);
                    stage = stage - 1;
                else
                    delay = delay + propDelay(stage+1);
                    stage = stage + 1;
                end
                direction = ~direction;
            end
            if stage == 0
                break;
            elseif delay > maxDelay | stage > numStages | stage < 0
                validPath = false;
                break
            end
        end
        if validPath & stage == 0
            % Avoid direct reflections
            if numel(find(pathsLogical(i,1:j) == 0)) == 1
                continue
            end
            % Avoid duplicate paths
            duplicate = false;
            for k=1:numel(completedPaths)
                if numel(completedPaths{k}) == j & completedPaths{k} == pathsLogical(i,1:j)
                    duplicate = true;
                end
            end
            if ~duplicate
                validPaths{end+1} = pathsLogical(i,1:j);
                completedPaths{end+1} = pathsLogical(i,1:j);
            end
        end
    end
    paths = validPaths;
end