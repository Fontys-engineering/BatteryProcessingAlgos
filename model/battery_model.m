classdef battery_model < matlab.System
    % battery_model Add summary here

    % Public, non-tunable properties
    properties (Nontunable)
        NUM_IMPEDANCES = 3;
        REFLECT_COEF = [0.5 0.5 0.5];
        PROPAGATION_DELAY = 200e-9;
        STEP_TIME = 2.5e-12; % TODO: make programmatic
        % DELAY_LENGTH = int32(PROPAGATION_DELAY / STEP_TIME);
        DELAY_LENGTH
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
            obj.S11 = obj.REFLECT_COEF;
            obj.S21 = 1 + obj.REFLECT_COEF;
            obj.S12 = 1 - obj.REFLECT_COEF;
            obj.S22 = -obj.REFLECT_COEF;
        end

        function reflection = stepImpl(obj, input)
            internal_transmit = [input 0];
            for i=1:obj.NUM_IMPEDANCES
                % STAGE
                internal_transmit(2) = obj.transmit_delay(i,obj.DELAY_LENGTH(i))*obj.S21(i) + obj.reflect_delay(i+1,1)*obj.S22(i);
                internal_reflect = obj.transmit_delay(i,obj.DELAY_LENGTH(i))*obj.S11(i) + obj.reflect_delay(i+1,1)*obj.S12(i);

                % RX DELAY
                obj.reflect_delay(i,1:obj.DELAY_LENGTH(i)) = [obj.reflect_delay(i,2:obj.DELAY_LENGTH(i)) internal_reflect];

                % TX DELAY
                obj.transmit_delay(i,1:obj.DELAY_LENGTH(i)) = [internal_transmit(1) obj.transmit_delay(i,1:obj.DELAY_LENGTH(i)-1)];
                internal_transmit(1) = internal_transmit(2);
            end
            reflection = obj.reflect_delay(1,1);
        end

        function resetImpl(obj)
            obj.transmit_delay = zeros(obj.NUM_IMPEDANCES+1, max(obj.DELAY_LENGTH));
            obj.reflect_delay = zeros(obj.NUM_IMPEDANCES+1, max(obj.DELAY_LENGTH));
        end
    end
end
