reflectionCoefsString = ["1/3","1/10","1"];
reflectionCoefs = [1/3 1/10 1];
delayLengths = [100 100 100];
numImpedances = length(reflectionCoefs);
stepTime = 2.5e-12;
stopTime = 10e-9;
numSteps = floorDiv(stopTime,stepTime)+1;

simIn = Simulink.SimulationInput("battery_verification_model");
for i=1:numImpedances
    simIn = simIn.setBlockParameter(sprintf("battery_verification_model/transmit_delay%d",i),"DelayLength",string(delayLengths(i)));
    simIn = simIn.setBlockParameter(sprintf("battery_verification_model/reflect_delay%d",i),"DelayLength",string(delayLengths(i)));
end
for i=1:numImpedances-1
    simIn = simIn.setBlockParameter(sprintf("battery_verification_model/stage%d/S11",i),"Gain",reflectionCoefsString(i));
    simIn = simIn.setBlockParameter(sprintf("battery_verification_model/stage%d/S21",i),"Gain","1+"+reflectionCoefsString(i));
    simIn = simIn.setBlockParameter(sprintf("battery_verification_model/stage%d/S12",i),"Gain","1-"+reflectionCoefsString(i));
    simIn = simIn.setBlockParameter(sprintf("battery_verification_model/stage%d/S22",i),"Gain","-"+reflectionCoefsString(i));
end
simIn = simIn.setBlockParameter(sprintf("battery_verification_model/stage%d",numImpedances),"Gain",reflectionCoefsString(numImpedances));

simIn = simIn.setModelParameter("FixedStep",string(stepTime));
simIn = simIn.setModelParameter("StopTime",string(stopTime));
out = sim(simIn);

validationData = squeeze(out.yout{1}.Values.Data);

dut = battery_model("NUM_IMPEDANCES",numImpedances,"REFLECT_COEF",reflectionCoefs,...
                    "DELAY_LENGTH",delayLengths);
dutOut = zeros(numSteps,1);
for i=1:numSteps
    dutOut(i) = dut(double(i==1));
end

if isequal(dutOut,validationData)
    disp("Pass")
else
    disp("Fail")
end