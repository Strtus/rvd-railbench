classdef RvdLabHillDynamics < matlab.System
    % Integrate a CWH relative-motion model from measured Hill position.

    properties (Nontunable)
        Period (1,1) double {mustBePositive} = 1
        MeanMotion (1,1) double {mustBePositive} = 0.0011
        PositionGain (1,1) double {mustBePositive} = 0.20
        VelocityGain (1,1) double {mustBePositive} = 0.80
    end

    properties (DiscreteState)
        ModelVelocity
    end

    methods (Access = protected)
        function command = stepImpl(obj, measuredPosition)
            position = reshape(measuredPosition, 3, 1);
            velocity = obj.ModelVelocity;
            n = obj.MeanMotion;

            % The target is the Hill origin. The PD loop uses encoder position
            % and the model's velocity state. The CWH drift is integrated here.
            control = -obj.PositionGain * position - obj.VelocityGain * velocity;
            drift = [3*n^2*position(1) + 2*n*velocity(2); ...
                     -2*n*velocity(1); -n^2*position(3)];
            acceleration = control + drift;
            dt = obj.Period;

            nextPosition = position + dt*velocity + 0.5*dt^2*acceleration;
            nextVelocity = velocity + dt*acceleration;
            obj.ModelVelocity = nextVelocity;
            % A positioning drive uses the average speed of this segment.
            command = [nextPosition; (nextPosition - position)/dt];
        end

        function resetImpl(obj)
            obj.ModelVelocity = zeros(3, 1);
        end

        function [size, dataType, complexity] = getDiscreteStateSpecificationImpl(~, ~)
            size = [3, 1];
            dataType = 'double';
            complexity = false;
        end

        function sampleTime = getSampleTimeImpl(obj)
            sampleTime = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Period);
        end

        function size = getOutputSizeImpl(~)
            size = [6, 1];
        end

        function dataType = getOutputDataTypeImpl(~)
            dataType = 'double';
        end

        function flag = isOutputComplexImpl(~)
            flag = false;
        end

        function flag = isOutputFixedSizeImpl(~)
            flag = true;
        end
    end
end
