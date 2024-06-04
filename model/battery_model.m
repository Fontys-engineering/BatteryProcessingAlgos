classdef battery_model < matlab.System
    % battery_model Add summary here

    % Public, non-tunable properties
    properties (Nontunable)
        NUM_IMPEDANCES = 3;
        IMPEDANCES = [50 100 200 300]
        STEP_TIME = 2.5e-12; % TODO: make programmatic
        DELAY_LENGTH_TIME
    end

    % Discrete state properties
    properties (DiscreteState)
        transmit_delay
        reflect_delay
    end

    % Pre-computed constants or internal states
    properties (Access = private)
        S11
        S12
        S21
        S22
        delayLength
    end

    methods
        % Constructor
        function obj = battery_model(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            for i=1:obj.NUM_IMPEDANCES
                obj.S11(i) = (obj.IMPEDANCES(i)-obj.IMPEDANCES(i+1))/(obj.IMPEDANCES(i)+obj.IMPEDANCES(i+1));
                obj.delayLength(i) = int32(obj.DELAY_LENGTH_TIME(i)/obj.STEP_TIME);
            end
            obj.S21 = 1 + obj.S11;
            obj.S12 = 1 - obj.S11;
            obj.S22 = -obj.S11;
        end

        function reflection = stepImpl(obj, input)
            internal_transmit = [input 0];
            reflection = obj.reflect_delay(1,1);
            for i=1:obj.NUM_IMPEDANCES
                % STAGE
                internal_transmit(2) = obj.transmit_delay(i,obj.delayLength(i))*obj.S21(i) + obj.reflect_delay(i+1,1)*obj.S22(i);
                internal_reflect = obj.transmit_delay(i,obj.delayLength(i))*obj.S11(i) + obj.reflect_delay(i+1,1)*obj.S12(i);

                % RX DELAY
                obj.reflect_delay(i,1:obj.delayLength(i)) = [obj.reflect_delay(i,2:obj.delayLength(i)) internal_reflect];

                % TX DELAY
                obj.transmit_delay(i,1:obj.delayLength(i)) = [internal_transmit(1) obj.transmit_delay(i,1:obj.delayLength(i)-1)];
                internal_transmit(1) = internal_transmit(2);
            end
        end

        function resetImpl(obj)
            obj.transmit_delay = zeros(obj.NUM_IMPEDANCES+1, max(obj.delayLength));
            obj.reflect_delay = zeros(obj.NUM_IMPEDANCES+1, max(obj.delayLength));
        end
    end
end
