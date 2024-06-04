function tests = battery_model_test
    tests = functiontests(localfunctions);
end

function setup(testCase)
    % Specify as string to avoid precission error
    testCase.TestData.impedances = ["50" "25" "225/11" "0"; "50" "30" "30/13" "0";
                                    "50" "300" "39" "76.5"; "100" "200" "400" "1"];
    testCase.TestData.delayLengths = [250e-12 250e-12 250e-12; 125e-12 190e-12 222.5e-12;
                                      85e-12 87.5e-12 90e-12; 82.5e-12 110e-12 137.5e-12];
    testCase.TestData.stepTime = 2.5e-12;
    testCase.TestData.stopTime = 5e-9;
end

function test_with_model(testCase)
    delayLengths = testCase.TestData.delayLengths;
    impedancesString = testCase.TestData.impedances;
    impedances = arrayfun(@str2num, impedancesString);
    stepTime = testCase.TestData.stepTime;
    stopTime = testCase.TestData.stopTime;
    numStages = length(impedances)-1;
    numSteps = floorDiv(stopTime,stepTime)+1;

    numSims = size(impedances,1);
    for i=1:numSims
        % Fill modelname directly bc matlab dependency analyzer is garbage
        simIn = Simulink.SimulationInput("battery_verification_model");
        modelName = simIn.ModelName;
        for j=1:numStages
            delayLengthsUnit(j) = delayLengths(i,j)/stepTime;
        end
        for j=1:numStages
            simIn = simIn.setBlockParameter(sprintf(modelName+"/transmit_delay%d",j),...
                                            "DelayLength",string(delayLengthsUnit(j)));
            simIn = simIn.setBlockParameter(sprintf(modelName+"/reflect_delay%d",j),...
                                            "DelayLength",string(delayLengthsUnit(j)));
        end

        for j = 1:numStages
            % Calculate S-parameters as string to avoid losing precision
            S11String(j) = "("+impedancesString(i,j+1)+"-"+impedancesString(i,j)+")/("+...
                           impedancesString(i,j+1)+"+"+impedancesString(i,j)+")";
        end
        for j=1:numStages-1
            simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S11",j),...
                                            "Gain",S11String(j));
            simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S21",j),...
                                            "Gain","1+"+S11String(j));
            simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S12",j),...
                                            "Gain","1-"+S11String(j));
            simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d/S22",j),...
                                            "Gain","-"+S11String(j));
        end
        simIn = simIn.setBlockParameter(sprintf(modelName+"/stage%d",j+1),...
                                        "Gain",S11String(j+1));

        simIn = simIn.setBlockParameter(modelName+"/source",...
                                        "SampleTime",string(stepTime));
        simIn = simIn.setModelParameter("FixedStep",string(stepTime));
        simIn = simIn.setModelParameter("StopTime",string(stopTime));

        out = sim(simIn);
        validationData = squeeze(out.yout{1}.Values.Data);

        dut = battery_model("NUM_IMPEDANCES",numStages,"IMPEDANCES",...
                            impedances(i,:),"DELAY_LENGTH_TIME",delayLengths(i,:));
        dutOut = zeros(numSteps,1);
        for j=1:numSteps
            dutOut(j) = dut(double(j==1));
        end

        verifyEqual(testCase,dutOut,validationData)
    end
end

function test_with_spice(testCase)
    impedances = arrayfun(@str2num,testCase.TestData.impedances(1,:));
    delayLengths = testCase.TestData.delayLengths(1,:);
    numStages = numel(delayLengths);
    stepTime = testCase.TestData.stepTime;
    stopTime = testCase.TestData.stopTime;
    numSteps = int32(stopTime/stepTime)+1;

    modelName = 'model'; % Exclude file extension
    ltspicePath = 'C:/"Program Files"/LTC/LTspiceXVII/XVIIx64.exe';
    system(append(ltspicePath,' --netlist ',modelName,'.asc'));
    system(append(ltspicePath,' -b ',modelName,'.net'));
    spiceData = LTspice2Matlab(append(modelName,'.raw'));

    dut = battery_model("NUM_IMPEDANCES",numStages,"IMPEDANCES",...
                        impedances,"DELAY_LENGTH_TIME",delayLengths);
    dutOut = zeros(numSteps,1);
    for j=1:numSteps
        dutOut(j) = dut(double(j==1));
    end

    hold on
    plot(spiceData.time_vect,spiceData.variable_mat)
    plot(0:stepTime:stopTime,dutOut)
    hold off
    xlabel("Time (s)")
    legend(["LTspice" "DUT"])
    testCase.verifyTrue(true)
end