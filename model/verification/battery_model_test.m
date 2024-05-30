clear
clc

% Specify as string to avoid precission error
reflectionCoefsString = ["1/3" "1/10" "1"];
delayLengths = [100 100 100];
stepTime = 2.5e-12;
stopTime = 10e-9;
modelName = "battery_verification_model";

reflectionCoefs = arrayfun(@str2num,reflectionCoefsString);
numImpedances = length(reflectionCoefs);
numSteps = floorDiv(stopTime,stepTime)+1;

simIn = Simulink.SimulationInput(modelName);
for i=1:numImpedances
    simIn = simIn.setBlockParameter(sprintf(modelName+"/transmit_delay%d",i),...
                                    "DelayLength",string(delayLengths(i)));
    simIn = simIn.setBlockParameter(sprintf(modelName+"/reflect_delay%d",i),...
                                    "DelayLength",string(delayLengths(i)));
end
for i=1:numImpedances-1
    simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S11",i),...
                                    "Gain",reflectionCoefsString(i));
    simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S21",i),...
                                    "Gain","1+"+reflectionCoefsString(i));
    simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S12",i),...
                                    "Gain","1-"+reflectionCoefsString(i));
    simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S22",i),...
                                    "Gain","-"+reflectionCoefsString(i));
end
simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d",i+1),...
                                "Gain",reflectionCoefsString(i+1));
simIn = simIn.setBlockParameter(modelName+"/source",...
                                "SampleTime",string(stepTime));

simIn = simIn.setModelParameter("FixedStep",string(stepTime));
simIn = simIn.setModelParameter("StopTime",string(stopTime));
out = sim(simIn);
validationData = squeeze(out.yout{1}.Values.Data);

dut = battery_model("NUM_IMPEDANCES",numImpedances,"REFLECT_COEF",...
                    reflectionCoefs,"DELAY_LENGTH",delayLengths);
dutOut = zeros(numSteps,1);
for i=1:numSteps
    dutOut(i) = dut(double(i==1));
end

if isequal(dutOut,validationData)
    disp("Pass")
else
    disp("Fail")
end