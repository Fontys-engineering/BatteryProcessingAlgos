clear
clc

% Specify as string to avoid precission error
reflectionCoefsString = ["1/3" "1/10" "1"; "1/4" "6/7" "1";
                         "8/10" "0.765" "1"; "0.1" "0.1" "0.1"];
delayLengths = [100 100 100; 50 76 89; 34 35 36; 33 44 55];
stepTime = 2.5e-12;
stopTime = 5e-9;
modelName = "battery_verification_model";

numSims = size(reflectionCoefsString,1);
for i=1:numSims
    reflectionCoefs = arrayfun(@str2num,reflectionCoefsString(i,:));
    numImpedances = length(reflectionCoefs);
    numSteps = floorDiv(stopTime,stepTime)+1;

    simIn = Simulink.SimulationInput(modelName);
    for j=1:numImpedances
        simIn = simIn.setBlockParameter(sprintf(modelName+"/transmit_delay%d",j),...
                                        "DelayLength",string(delayLengths(i,j)));
        simIn = simIn.setBlockParameter(sprintf(modelName+"/reflect_delay%d",j),...
                                        "DelayLength",string(delayLengths(i,j)));
    end
    for j=1:numImpedances-1
        simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S11",j),...
                                        "Gain",reflectionCoefsString(i,j));
        simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S21",j),...
                                        "Gain","1+"+reflectionCoefsString(i,j));
        simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S12",j),...
                                        "Gain","1-"+reflectionCoefsString(i,j));
        simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S22",j),...
                                        "Gain","-"+reflectionCoefsString(i,j));
    end
    simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d",j+1),...
                                    "Gain",reflectionCoefsString(i,j+1));
    simIn = simIn.setBlockParameter(modelName+"/source",...
                                    "SampleTime",string(stepTime));

    simIn = simIn.setModelParameter("FixedStep",string(stepTime));
    simIn = simIn.setModelParameter("StopTime",string(stopTime));
    out = sim(simIn);
    validationData = squeeze(out.yout{1}.Values.Data);

    dut = battery_model("NUM_IMPEDANCES",numImpedances,"REFLECT_COEF",...
                        reflectionCoefs,"DELAY_LENGTH",delayLengths(i,:));
    dutOut = zeros(numSteps,1);
    for j=1:numSteps
        dutOut(j) = dut(double(j==1));
    end

    if isequal(dutOut,validationData)
        disp("Pass")
    else
        disp("Fail")
    end
end