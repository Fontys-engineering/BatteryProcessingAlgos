function tests = battery_model_test
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Specify as string to avoid precission error
    testCase.TestData.impedances = ["50" "25" "225/11" "0"; "50" "30" "30/13" "0";
                                    "50" "300" "39" "76.5"; "100" "200" "400" "1"];
    testCase.TestData.delayLengths = [250e-12 250e-12 250e-12; 125e-12 190e-12 222.5e-12;
                                      85e-12 87.5e-12 90e-12; 82.5e-12 110e-12 137.5e-12];
    testCase.TestData.stepTime = 100e-15;
    testCase.TestData.stopTime = 5e-9;
    numSteps = int32(testCase.TestData.stopTime/testCase.TestData.stepTime)+1;

    % Get DUT data
    numSims = size(testCase.TestData.delayLengths,1);
    numStages = size(testCase.TestData.delayLengths,2);
    for i=1:numSims
        dut = battery_model("NUM_IMPEDANCES",numStages,...
                            "IMPEDANCES",arrayfun(@str2num,testCase.TestData.impedances(i,:)),...
                            "DELAY_LENGTH_TIME",testCase.TestData.delayLengths(i,:),...
                            "STEP_TIME", testCase.TestData.stepTime);
        dutOut(i,:) = zeros(1,numSteps);
        for j=1:numSteps
            dutOut(i,j) = dut(double(j==1));
        end
    end
    testCase.TestData.dutOut = dutOut;
end

function test_with_model(testCase)
    delayLengths = testCase.TestData.delayLengths;
    impedancesString = testCase.TestData.impedances;
    impedances = arrayfun(@str2num, impedancesString);
    stepTime = testCase.TestData.stepTime;
    stopTime = testCase.TestData.stopTime;
    numStages = length(impedances)-1;

    numSims = size(impedances,1);
    for i=1:numSims
        % Fill modelname directly bc matlab dependency analyzer is garbage
        simIn = Simulink.SimulationInput("battery_verification_model");
        modelName = simIn.ModelName;
        for j=1:numStages
            delayLengthsUnit(j) = int32(delayLengths(i,j)/stepTime);
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
        validationData = squeeze(out.yout{1}.Values.Data)';
        dutOut = testCase.TestData.dutOut(i,:);

        testCase.verifyEqual(dutOut,validationData)
    end
end

function test_with_spice(testCase)
    dutOut = testCase.TestData.dutOut;
    stepTime = testCase.TestData.stepTime;
    stopTime = testCase.TestData.stopTime;
    dutTime = 0:stepTime:stopTime;
    impedances = testCase.TestData.impedances;
    delayLength = testCase.TestData.delayLengths;

    modelName = "spice_model"; % Exclude file extension
    ltspicePath = "C:/""Program Files""/LTC/LTspiceXVII/XVIIx64.exe";
    numSims = size(impedances,1);
    for i=1:numSims
        % Create netlist
        template = fileread("model/verification/spice_model_template.net");
        netlistID = fopen("spice_model.net", "w");
        fprintf(netlistID,"%s",template);
        fprintf(netlistID,".PARAM simTime %e\n",stopTime);
        fprintf(netlistID,".PARAM imp1 %s\n",impedances(i,1));
        fprintf(netlistID,".PARAM imp2 %s\n",impedances(i,2));
        fprintf(netlistID,".PARAM imp3 %s\n",impedances(i,3));
        fprintf(netlistID,".PARAM imp4 %s\n",impedances(i,4));
        fprintf(netlistID,".PARAM pd1 %e\n",delayLength(i,1));
        fprintf(netlistID,".PARAM pd2 %e\n",delayLength(i,2));
        fprintf(netlistID,".PARAM pd3 %e\n",delayLength(i,3));
        fprintf(netlistID,".backanno\n.end\n");
        fclose(netlistID);
        system(ltspicePath+" -b "+modelName+".net");
        delete spice_model.net
        spiceData(i) = LTspice2Matlab(modelName+".raw");
    end

    for i=1:numSims
        subplot(2,2,i)
        hold on
        plot(spiceData(i).time_vect*1e9,spiceData(i).variable_mat)
        plot(dutTime*1e9,dutOut(i,:))
        hold off
        xlabel("Time (ns)")
        legend(["LTspice" "DUT"])
    end

    testCase.verifyTrue(true)
end