classdef RvdLabExternalIO < matlab.System
    % Exchange Hill-frame position and motion references with RVDLAB.

    properties (Nontunable)
        Port (1,1) double {mustBeInteger, mustBeNonnegative} = 0
        Period (1,1) double {mustBePositive, mustBeFinite} = 1
        Token (1,:) uint8 = zeros(1,0,'uint8')
        ExpectedSteps (1,1) double {mustBeInteger, mustBeNonnegative} = 0
    end

    properties (Access = private)
        Client = []
        LastReadTime = -Inf
        LastWriteTime = -Inf
        LastAcquired = -Inf
        LastFrame = -1
        Sequence = -1
        Position = zeros(3,1)
    end

    methods (Access = protected)
        function setupImpl(obj, ~)
            if obj.Port < 1 || obj.Port > 65535
                error("RvdLab:InvalidPort", "Use the port from rvdlab external or run the demo launcher.");
            end
            if isempty(obj.Token) || any(isspace(char(obj.Token)))
                error("RvdLab:MissingToken", "Use the authentication token from the private readiness file.");
            end
        end

        function resetImpl(obj)
            obj.LastReadTime = -Inf;
            obj.LastWriteTime = -Inf;
            obj.LastAcquired = -Inf;
            obj.LastFrame = -1;
            obj.Sequence = -1;
            obj.Position = zeros(3,1);
        end

        function position = outputImpl(obj, ~)
            sampleTime = getCurrentTime(obj);
            if sampleTime ~= obj.LastReadTime
                if isempty(obj.Client)
                    obj.connect();
                end
                fields = obj.readFields();
                if fields(1) == "END"
                    error("RvdLab:ExternalStopped", "RVDLAB ended the experiment: %s", join(fields(2:end)));
                end
                if numel(fields) ~= 7 || fields(1) ~= "STATE"
                    error("RvdLab:InvalidState", "Unexpected state message: %s", join(fields));
                end
                values = str2double(fields(2:end));
                if any(~isfinite(values)) || values(1) ~= obj.Sequence + 1 || ...
                        values(2) <= obj.LastAcquired || values(3) <= obj.LastFrame || ...
                        values(3) ~= fix(values(3))
                    error("RvdLab:InvalidState", "Invalid or repeated state message: %s", join(fields));
                end
                obj.Sequence = values(1);
                obj.LastAcquired = values(2);
                obj.LastFrame = values(3);
                obj.Position = reshape(values(4:6), 3, 1);
                obj.LastReadTime = sampleTime;
                obj.Client.Timeout = 3*obj.Period;
            end
            position = obj.Position;
        end

        function updateImpl(obj, reference)
            sampleTime = getCurrentTime(obj);
            if sampleTime == obj.LastWriteTime
                return
            end
            validateattributes(reference, {'double'}, {'real','vector','numel',6,'finite'});
            writeline(obj.Client, sprintf("REF %d %.17g %.17g %.17g %.17g %.17g %.17g", ...
                obj.Sequence, reference(:)));
            obj.LastWriteTime = sampleTime;

            % Wait for the final tracking check and durable result.
            if obj.ExpectedSteps > 0 && obj.Sequence + 1 == obj.ExpectedSteps
                fields = obj.readFields();
                if numel(fields) ~= 3 || fields(1) ~= "END" || ...
                        fields(2) ~= "COMPLETE" || fields(3) ~= "MAX_STEPS"
                    error("RvdLab:ExternalStopped", "RVDLAB ended the experiment: %s", join(fields));
                end
            end
        end

        function releaseImpl(obj)
            obj.Client = [];
        end

        function direct = isInputDirectFeedthroughImpl(~)
            direct = false;
        end

        function sampleTime = getSampleTimeImpl(obj)
            sampleTime = createSampleTime(obj, Type="Discrete", SampleTime=obj.Period);
        end

        function sizeOut = getOutputSizeImpl(~)
            sizeOut = [3 1];
        end

        function typeOut = getOutputDataTypeImpl(~)
            typeOut = "double";
        end

        function isComplex = isOutputComplexImpl(~)
            isComplex = false;
        end

        function fixed = isOutputFixedSizeImpl(~)
            fixed = true;
        end
    end

    methods (Access = private)
        function connect(obj)
            obj.Client = tcpclient("127.0.0.1", obj.Port, Timeout=max(20, 3*obj.Period));
            configureTerminator(obj.Client, "LF");
            try
                writeline(obj.Client, "HELLO " + string(char(obj.Token)));
            catch
                obj.Client = [];
                error("RvdLab:HandshakeFailed", "Could not authenticate to the local RVDLAB gateway.");
            end
        end

        function fields = readFields(obj)
            data = readline(obj.Client);
            if isempty(data) || strlength(strtrim(data)) == 0
                error("RvdLab:Timeout", "RVDLAB did not send a message within the TCP timeout.");
            end
            fields = split(strtrim(data));
        end
    end
end
