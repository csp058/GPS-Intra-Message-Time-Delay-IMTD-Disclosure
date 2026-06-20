classdef gpsNavigationConfig < comm.internal.ConfigBase
    properties
        SignalType = "CNAV2"
        PRNID = 1
        L1CSubframe3PageSequence = [repmat(4,31,1); 7; 1; 2; 3; 8; repmat(5,10,1); repmat(6,3,1)]
        L1CTOI = 0
        L1CITOW = 0
        L1CHealth = 0 
        WeekNumber = 2149
        GroupDelayDifferential = 0 
        SemiMajorAxisLength = 26560000
        ChangeRateInSemiMajorAxis = 0
        MeanMotionDifference = 0
        RateOfMeanMotionDifference = 0
        Eccentricity = 0.02
        MeanAnomaly = 0
        ReferenceTimeOfEphemeris = 0 
        HarmonicCorrectionTerms = zeros(6,1) 
        IntegrityStatusFlag = 0
        ArgumentOfPerigee = -0.52
        RateOfRightAscension = 0
        LongitudeOfAscendingNode = -0.84
        Inclination = 0.3 
        InclinationRate = 0
        URAEDID = 0
        InterSignalCorrection = zeros(4,1) 
        ISCL1CP = 0
        ISCL1CD = 0
        ReferenceTimeCEIPropagation = 0 
        ReferenceWeekNumberCEIPropagation = 101 
        URANEDID = [0; 0 ; 0] 
        
        SVClockCorrectionCoefficients = [0; 0; 0] 
        
        AlmanacFileName = "gpsAlmanac.txt"
        Ionosphere = struct('Alpha',zeros(4,1),'Beta',zeros(4,1))
        EarthOrientation = struct('ReferenceTimeEOP',0,'XAxisPolarMotionValue',0,...
            'XAxisPolarMotionDrift',0,'YAxisPolarMotionValue',0, ...
            'YAxisPolarMotionDrift',0,'UT1_UTCDifference',0, ...
            'RateOfUT1_UTCDifference',0);
        UTC = struct('UTCTimeCoefficients',[0 0 0],'PastLeapSecondCount',18, ...
            'ReferenceTimeUTCData',0,'TimeDataReferenceWeekNumber', 2149, ...
            'LeapSecondReferenceWeekNumber',2149,'LeapSecondReferenceDayNumber',1, ...
            'FutureLeapSecondCount',18)
        DifferentialCorrection = struct('ReferenceTimeDCData',0, ...
            'ReferenceTimeDCDataPredict',0, ...
            'Data',repmat(struct('DCDataType',0,'CDCPRNID',1,'SVClockBiasCoefficient',0, ...
            'SVClockDriftCorrection',0,'UDRAID',1,'EDCPRNID',1, ...
            'AlphaCorrection',0,'BetaCorrection',0,'GammaCorrection',0, ...
            'InclinationCorrection',0,'RightAscensionCorrection',0, ...
            'SemiMajorAxisCorrection',0,'UDRARateID',0),31,1))
        TimeOffset = struct('ReferenceTimeGGTO',0,'WeekNumberGGTO',101, ...
            'GNSSID',0,'GGTOCoefficients',[0;0;0])
        ReducedAlmanac = struct('WeekNumber',1,'ReferenceTimeOfAlmanac',0, ...
            'Almanac',repmat(struct('PRNa',0,'delta_A',0,'Omega0',0, ...
            'Phi0',0,'L1Health',0,'L2Health',0,'L5Health',0),6,1)) 
        TextMessage = 'Enhanced Chimera CNAV-2 Payload Test          '
        ISM = struct('GNSSID',4, 'Weeknumber',1,'TOW',0,'CorrelationTimeConstantID',0, ...
            'AdditiveTermID',0,'ScalarTermID',0,'SatelliteFaultRateID',0, ...
            'ConstellationFaultProbabilityID',0,'MeanFaultDurationID',0, ...
            'ServiceLevel',1,'SatelliteInclusionMask',zeros(63,1), ...
            'CRCPolynomial','x^32 + x^31 + x^24 + x^22 + x^16 + x^14 + x^8 + x^7 + x^5 + x^3 + x + 1');
    end

    properties(Hidden)
        PageID 
        SubframeID 
    end

    methods
        function obj = gpsNavigationConfig(varargin)
            obj@comm.internal.ConfigBase(varargin{:});
        end

        function obj = set.SignalType(obj,val)
            obj.SignalType = string(val);
        end
        function obj = set.PRNID(obj,val)
            validateattributes(val,{'double','single','uint8'},{'positive','integer','scalar','>=',1,'<=',210},mfilename,'PRNID')
            obj.PRNID = val;
        end
        function obj = set.AlmanacFileName(obj,val)
            validateattributes(val,{'char','string'},{},mfilename,'AlmanacFileName')
            obj.AlmanacFileName = val;
        end
    end

    methods(Access = protected)
        function flag = isInactiveProperty(~,~)
            flag = false; 
        end
    end
end