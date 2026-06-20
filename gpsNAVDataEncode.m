function [bits, rawSF1, rawSF2, rawSF3] = gpsNAVDataEncode(cfg)
% custom_HelperGPSNAVDataEncode 
% Lightweight CNAV-2 (L1C) encoder for Enhanced Chimera.
% Outputs both the fully encoded frame AND the raw, uncoded subframes.

[~, almWeekNum, almTimeOfApplicability, almStruct] = ...
    matlabshared.internal.gnss.readSEMAlmanac(cfg.AlmanacFileName);

if ~isfield(almStruct,'L1Health'), [almStruct(:).L1Health] = deal(0); end
if ~isfield(almStruct,'L2Health'), [almStruct(:).L2Health] = deal(0); end
if ~isfield(almStruct,'L5Health'), [almStruct(:).L5Health] = deal(0); end

toi = mod(cfg.L1CTOI,400);

% Load Parity Check Matrices for Subframe 2
load("L1CLDPCParityCheckMatrices.mat","A1","B1","C1","E1","T1");
l1cLDPCA1 = logical(sparse(A1(:,1),A1(:,2),1,599,600));
l1cLDPCB1 = logical(sparse(B1(:,1),B1(:,2),1,599,1));
l1cLDPCC1 = logical(sparse(C1(:,1),C1(:,2),1,1,600));
l1cLDPCD1 = true;
l1cLDPCE1 = logical(sparse(E1(:,1),E1(:,2),1,1,599));
l1cLDPCT1 = logical(sparse(T1(:,1),T1(:,2),1,599,599));
cfgLDPCSubframe2 = ldpcEncoderConfig([l1cLDPCA1, l1cLDPCB1, l1cLDPCT1; l1cLDPCC1, l1cLDPCD1, l1cLDPCE1]);
crcGeneratorSubframe2 = comm.CRCGenerator('Polynomial', ...
    'z^24 + z^23 + z^18 + z^17 + z^14 + z^11 + z^10 + z^7 + z^6 + z^5 + z^4 + z^3 + z + 1');

s3PageSeq = cfg.L1CSubframe3PageSequence;
numPages = length(s3PageSeq(:));

s3Bits = zeros(548,numPages);
dTemp  = zeros(1800,numPages);

rawSF1 = zeros(52, numPages);
rawSF2 = zeros(600, numPages);
rawSF3 = zeros(274, numPages);

almCount = 1;
dcParamCount = 1;
dcParamStruct = cfg.DifferentialCorrection.Data;
textPageCount = 0;
fullText = cfg.TextMessage;
fullTextLength = length(fullText(:));
svconfig = [almStruct(:).SatelliteConfiguration];
svconfig = [svconfig;[almStruct(:).PRNNumber]];

% Load Parity Check Matrices for Subframe 3
load("L1CLDPCParityCheckMatrices.mat","A2","B2","C2","E2","T2");
l1cLDPCA2 = logical(sparse(A2(:,1),A2(:,2),1,273,274));
l1cLDPCB2 = logical(sparse(B2(:,1),B2(:,2),1,273,1));
l1cLDPCC2 = logical(sparse(C2(:,1),C2(:,2),1,1,274));
l1cLDPCD2 = true;
l1cLDPCE2 = logical(sparse(E2(:,1),E2(:,2),1,1,273));
l1cLDPCT2 = logical(sparse(T2(:,1),T2(:,2),1,273,273));
cfgLDPCSubframe3 = ldpcEncoderConfig([l1cLDPCA2, l1cLDPCB2, l1cLDPCT2; l1cLDPCC2, l1cLDPCD2, l1cLDPCE2]);
crcGeneratorSubframe3 = comm.CRCGenerator('Polynomial', ...
    'z^24 + z^23 + z^18 + z^17 + z^14 + z^11 + z^10 + z^7 + z^6 + z^5 + z^4 + z^3 + z + 1');

numcharPerPage = 29;
if fullTextLength<numcharPerPage, textIndices = 1:fullTextLength; else, textIndices = 1:numcharPerPage; end

for iPage = 1:numPages
    numBlanksInTextMessage = numcharPerPage - length(textIndices);
    textMessage = [fullText(textIndices), repmat(' ', 1, numBlanksInTextMessage)];
    
    % SF1 Processing
    toiBits = num2bits(toi,9,1);
    s1Bits = gpsTOIEnc(toiBits);
    rawSF1(:, iPage) = s1Bits; % 52 bits
    
    % SF2 Processing
    [s2Bits, rSF2] = l1cSubframe2(cfg,cfgLDPCSubframe2,crcGeneratorSubframe2);
    rawSF2(:, iPage) = rSF2;   % 600 bits
    
    % SF3 Processing
    [s3Bits(:,iPage), rSF3] = l1cSubframe3(cfg,s3PageSeq(iPage),almStruct(almCount), ...
        almWeekNum,almTimeOfApplicability,dcParamStruct(dcParamCount), ...
        textMessage,textPageCount,svconfig,cfgLDPCSubframe3,crcGeneratorSubframe3);
    rawSF3(:, iPage) = rSF3;   % 274 bits
    
    toi = mod(toi + 1, 400);
    if toi == 0, cfg.L1CITOW = mod(cfg.L1CITOW + 1, 84); end
    if s3PageSeq(iPage) == 4, almCount = mod(almCount,length(almStruct))+1; end
    if s3PageSeq(iPage) == 5, dcParamCount = mod(dcParamCount,length(dcParamStruct)) + 1; end
    if s3PageSeq(iPage) == 6
        if isempty(textIndices)
            tempIndices = [];
        else
            tempIndices = textIndices(end)+1:fullTextLength;
        end
        
        if length(tempIndices) > numcharPerPage
            textIndices = tempIndices(1:numcharPerPage); 
        else
            textIndices = tempIndices; 
        end
        textPageCount = mod(textPageCount+1,16);
    end

    s2s3 = matintrlv([s2Bits;s3Bits(:,iPage)],38,46);
    dTemp(:,iPage) = [s1Bits;s2s3];
end
bits = dTemp(:); 
end

function [s2Bits, raw_bits] = l1cSubframe2(cfg,cfgLDPC,crcgenerator)
WN = num2bits(cfg.WeekNumber,13,1);
ITOW = num2bits(cfg.L1CITOW,8,1);
t_op = num2bits(cfg.ReferenceTimeCEIPropagation,11,300);
L1CHealth = cfg.L1CHealth;
URAEDID = num2bits(cfg.URAEDID,5,1);
t_oe = num2bits(cfg.ReferenceTimeOfEphemeris,11,300);
A = cfg.SemiMajorAxisLength;
DeltaA = num2bits(A - 26559710,26,2^-9);
A_Dot = num2bits(cfg.ChangeRateInSemiMajorAxis,25,2^-21);
Deltan0 = num2bits(cfg.MeanMotionDifference,17,2^-44);
Deltan0Dot = num2bits(cfg.RateOfMeanMotionDifference,23,2^-57);
M_0 = num2bits(cfg.MeanAnomaly,33,2^-32);
e = num2bits(cfg.Eccentricity,33,2^-34);
omega = num2bits(cfg.ArgumentOfPerigee,33,2^-32);
Omega = num2bits(cfg.LongitudeOfAscendingNode,33,2^-32);
DeltaOmegaDot = num2bits(cfg.RateOfRightAscension+2.6e-9,17,2^-44); 
i0 = num2bits(cfg.Inclination,33,2^-32);
iDOT = num2bits(cfg.InclinationRate,15,2^-44);
Cis = num2bits(cfg.HarmonicCorrectionTerms(1),16,2^-30);
Cic = num2bits(cfg.HarmonicCorrectionTerms(2),16,2^-30);
Crs = num2bits(cfg.HarmonicCorrectionTerms(3),24,2^-8);
Crc = num2bits(cfg.HarmonicCorrectionTerms(4),24,2^-8);
Cus = num2bits(cfg.HarmonicCorrectionTerms(5),21,2^-30);
Cuc = num2bits(cfg.HarmonicCorrectionTerms(6),21,2^-30);
URANED0ID = num2bits(cfg.URANEDID(1),5,1);
URANED1ID = num2bits(cfg.URANEDID(2),3,1);
URANED2ID = num2bits(cfg.URANEDID(3),3,1);
af0 = num2bits(cfg.SVClockCorrectionCoefficients(1),26,2^-35);
af1 = num2bits(cfg.SVClockCorrectionCoefficients(2),20,2^-48);
af2 = num2bits(cfg.SVClockCorrectionCoefficients(3),10,2^-60);
T_GD = num2bits(cfg.GroupDelayDifferential,13,2^-35);
ISC_L1CP = num2bits(cfg.ISCL1CP,13,2^-35);
ISC_L1CD = num2bits(cfg.ISCL1CD,13,2^-35);
isc = cfg.IntegrityStatusFlag;
WN_op = num2bits(cfg.ReferenceWeekNumberCEIPropagation,8,1);

d = [WN;ITOW;t_op;L1CHealth;URAEDID;t_oe;DeltaA;A_Dot;Deltan0;Deltan0Dot; ...
    M_0;e;omega;Omega;i0;DeltaOmegaDot;iDOT;Cis;Cic;Crs;Crc;Cus;Cuc;URANED0ID; ...
    URANED1ID;URANED2ID;af0;af1;af2;T_GD;ISC_L1CP;ISC_L1CD;isc;WN_op;0;0]; 
raw_bits = crcgenerator(d); % Outputs 600 bits (Data + CRC)
s2Bits = ldpcEncode(raw_bits,cfgLDPC);
end

function [s3Bits, raw_bits] = l1cSubframe3(cfg,pageID,almStruct,almWeekNum, ...
    almTimeOfApplicability,dcParamStruct,textMessage,textPageCount,svconfig, ...
    cfgLDPC,crcgenerator)

prnid = num2bits(cfg.PRNID,8,1);
pageNum = num2bits(pageID,6,1);
switch(pageID)
    case 1
        A0 = num2bits(cfg.UTC.UTCTimeCoefficients(1),16,2^-35);
        A1 = num2bits(cfg.UTC.UTCTimeCoefficients(2),13,2^-51);
        A2 = num2bits(cfg.UTC.UTCTimeCoefficients(3),7,2^-68);
        Deltat_LS = num2bits(cfg.UTC.PastLeapSecondCount,8,1);
        t_ot = num2bits(cfg.UTC.ReferenceTimeUTCData,16,16);
        WN_ot = num2bits(cfg.UTC.TimeDataReferenceWeekNumber,13,1);
        WN_LSF = num2bits(cfg.UTC.LeapSecondReferenceWeekNumber,13,1);
        DN = num2bits(cfg.UTC.LeapSecondReferenceDayNumber,4,1);
        Deltat_LSF = num2bits(cfg.UTC.FutureLeapSecondCount,8,1);
        a0 = num2bits(cfg.Ionosphere.Alpha(1),8,2^-30);
        a1 = num2bits(cfg.Ionosphere.Alpha(2),8,2^-27);
        a2 = num2bits(cfg.Ionosphere.Alpha(3),8,2^-24);
        a3 = num2bits(cfg.Ionosphere.Alpha(4),8,2^-24);
        b0 = num2bits(cfg.Ionosphere.Beta(1),8,2^11);
        b1 = num2bits(cfg.Ionosphere.Beta(2),8,2^14);
        b2 = num2bits(cfg.Ionosphere.Beta(3),8,2^16);
        b3 = num2bits(cfg.Ionosphere.Beta(4),8,2^16);
        iscL1CA = num2bits(cfg.InterSignalCorrection(1),13,2^-35);
        iscL2C = num2bits(cfg.InterSignalCorrection(2),13,2^-35);
        iscL5I5 = num2bits(cfg.InterSignalCorrection(3),13,2^-35);
        iscL5Q5 = num2bits(cfg.InterSignalCorrection(4),13,2^-35);
        d = [A0;A1;A2;Deltat_LS;t_ot;WN_ot;WN_LSF;DN;Deltat_LSF; ...
            a0;a1;a2;a3;b0;b1;b2;b3;iscL1CA;iscL2C;iscL5I5;iscL5Q5;zeros(22,1)];
    case 2
        GNSSID = num2bits(cfg.TimeOffset.GNSSID,3,1);
        t_GGTO = num2bits(cfg.TimeOffset.ReferenceTimeGGTO,16,16);
        WN_GGTO = num2bits(cfg.TimeOffset.WeekNumberGGTO,13,1);
        A0GGTO = num2bits(cfg.TimeOffset.GGTOCoefficients(1),16,2^-35);
        A1GGTO = num2bits(cfg.TimeOffset.GGTOCoefficients(2),13,2^-51);
        A2GGTO = num2bits(cfg.TimeOffset.GGTOCoefficients(3),7,2^-68);
        t_EOP = num2bits(cfg.EarthOrientation.ReferenceTimeEOP,16,16);
        PM_X  = num2bits(cfg.EarthOrientation.XAxisPolarMotionValue,21,2^-20);
        PM_XRate  = num2bits(cfg.EarthOrientation.XAxisPolarMotionDrift,15,2^-21);
        PM_Y  = num2bits(cfg.EarthOrientation.YAxisPolarMotionValue,21,2^-20);
        PM_YRate  = num2bits(cfg.EarthOrientation.YAxisPolarMotionDrift,15,2^-21);
        DeltaUTGPS = num2bits(cfg.EarthOrientation.UT1_UTCDifference,31,2^-23);
        DeltaUTGPSRate = num2bits(cfg.EarthOrientation.RateOfUT1_UTCDifference,19,2^-25);
        d = [GNSSID;t_GGTO;WN_GGTO;A0GGTO;A1GGTO;A2GGTO;t_EOP;PM_X; ...
            PM_XRate;PM_Y;PM_YRate;DeltaUTGPS;DeltaUTGPSRate;zeros(30,1)];
    case 3
        WN_a = num2bits(cfg.ReducedAlmanac.WeekNumber,13,1);
        t_oa = num2bits(cfg.ReducedAlmanac.ReferenceTimeOfAlmanac,8,2^12);
        numReducedAlmanacPackets = 6; 
        almPackets = zeros(33,numReducedAlmanacPackets);
        for iAlm = 1:numReducedAlmanacPackets
            if cfg.ReducedAlmanac.Almanac(iAlm).PRNa == 0 
                numAlmPacketsToFill = numReducedAlmanacPackets-iAlm;
                numFillBits = numAlmPacketsToFill*33 + 25; 
                fillBits = [zeros(8,1);repmat([1;0],floor(numFillBits/2),1);zeros(mod(numFillBits,2),1)];
                almPackets(:,iAlm:end) = reshape(fillBits,33,[]);
                break;
            end
            PRN_a = num2bits(cfg.ReducedAlmanac.Almanac(iAlm).PRNa,8,1);
            delta_A = num2bits(cfg.ReducedAlmanac.Almanac(iAlm).delta_A,8,2^9);
            Omega0 = num2bits(cfg.ReducedAlmanac.Almanac(iAlm).Omega0,7,2^-6);
            Phi0 = num2bits(cfg.ReducedAlmanac.Almanac(iAlm).Phi0,7,2^-6);
            L1Health = cfg.ReducedAlmanac.Almanac(iAlm).L1Health;
            L2Health = cfg.ReducedAlmanac.Almanac(iAlm).L2Health;
            L5Health = cfg.ReducedAlmanac.Almanac(iAlm).L5Health;
            almPackets(:,iAlm) = [PRN_a;delta_A;Omega0;Phi0;L1Health;L2Health;L5Health];
        end
        d = [WN_a;t_oa;almPackets(:);zeros(17,1)];
    case 4 
        WN_a = num2bits(almWeekNum,13,1);
        t_oa = num2bits(almTimeOfApplicability,8,2^12);
        PRNa = num2bits(almStruct.PRNNumber,8,1);
        L1Health = almStruct.L1Health;
        L2Health = almStruct.L2Health;
        L5Health = almStruct.L5Health;
        e = num2bits(almStruct.Eccentricity,11,2^-16);
        deltai0 = num2bits(almStruct.InclinationOffset,11,2^-14);
        OmegaDot = num2bits(almStruct.RateOfRightAscension,11,2^-33);
        sqrtA = num2bits(almStruct.SqrtOfSemiMajorAxis,17,2^-4);
        Omega = num2bits(almStruct.GeographicLongitudeOfOrbitalPlane,16,2^-15);
        omega = num2bits(almStruct.ArgumentOfPerigee,16,2^-15);
        M0 = num2bits(almStruct.MeanAnomaly,16,2^-15);
        af0 = num2bits(almStruct.ZerothOrderClockCorrection,11,2^-20);
        af1 = num2bits(almStruct.FirstOrderClockCorrection,10,2^-37);
        d = [WN_a;t_oa;PRNa;L1Health;L2Health;L5Health;e;deltai0;OmegaDot; ...
            sqrtA;Omega;omega;M0;af0;af1;zeros(85,1)];
    case 5
        t_opD = num2bits(cfg.DifferentialCorrection.ReferenceTimeDCDataPredict,11,300);
        t_OD  = num2bits(cfg.DifferentialCorrection.ReferenceTimeDCData,11,300);
        dcDataType = dcParamStruct.DCDataType;
        cdcPRN = num2bits(dcParamStruct.CDCPRNID,8,1);
        delta_af0 = num2bits(dcParamStruct.SVClockBiasCoefficient,13,2^-35);
        delta_af1 = num2bits(dcParamStruct.SVClockDriftCorrection,8,2^-51);
        UDRAID = num2bits(dcParamStruct.UDRAID,5,1);
        edcPRN = num2bits(dcParamStruct.EDCPRNID,8,1);
        Deltaa = num2bits(dcParamStruct.AlphaCorrection,14,2^-34);
        Deltab = num2bits(dcParamStruct.BetaCorrection,14,2^-34);
        Deltay = num2bits(dcParamStruct.GammaCorrection,15,2^-32);
        Deltai = num2bits(dcParamStruct.InclinationCorrection,12,2^-32);
        DeltaO = num2bits(dcParamStruct.RightAscensionCorrection,12,2^-32);
        DeltaA = num2bits(dcParamStruct.SemiMajorAxisCorrection,12,2^-9);
        UDRARateID = num2bits(dcParamStruct.UDRARateID,5,1);
        d = [t_opD;t_OD;dcDataType;cdcPRN;delta_af0;delta_af1;UDRAID; ...
            edcPRN;Deltaa;Deltab;Deltay;Deltai;DeltaO;DeltaA;UDRARateID;zeros(87,1)];
    case 6
        textPageID = num2bits(textPageCount,4,1);
        textMessageBits = int2bit(double(char(textMessage(:))),8);
        d = [textPageID;textMessageBits];
    case 7
        svconfigbits = zeros(3,63);
        cnt = 0;
        for iprn = svconfig(2,:)
            cnt = cnt + 1;
            svconfigbits(:,iprn) = int2bit(svconfig(1,cnt),3);
        end
        d = [svconfigbits(:);zeros(47,1)];
    case 8 
        GNSSID = num2bits(cfg.ISM.GNSSID,4,1);
        wn_ISM = num2bits(cfg.ISM.Weeknumber,13,1);
        tow_ISM = num2bits(cfg.ISM.TOW,6,4);
        t_correl = num2bits(cfg.ISM.CorrelationTimeConstantID,4,1);
        b_norm = num2bits(cfg.ISM.AdditiveTermID,4,1);
        gamma_norm = num2bits(cfg.ISM.ScalarTermID,4,1);
        R_sat = num2bits(cfg.ISM.SatelliteFaultRateID,4,1);
        P_const = num2bits(cfg.ISM.ConstellationFaultProbabilityID,4,1);
        MFD = num2bits(cfg.ISM.MeanFaultDurationID,4,1);
        serviceLevel = int2bit(cfg.ISM.ServiceLevel-1,3); 
        mask = cfg.ISM.SatelliteInclusionMask(:);
        tempfiller = repmat([1;0],1,50);
        filler = tempfiller(1:91);
        ismcrcgen = comm.CRCGenerator(cfg.ISM.CRCPolynomial);
        ismBits = [GNSSID;wn_ISM;tow_ISM;t_correl;b_norm;gamma_norm;R_sat;P_const; ...
            MFD;serviceLevel;mask;filler(:)];
        d = ismcrcgen(ismBits);
    otherwise
end
raw_bits = crcgenerator([prnid;pageNum;d]); % Outputs 274 bits (Data + CRC)
s3Bits = ldpcEncode(raw_bits,cfgLDPC);
end

function y = gpsTOIEnc(x)
msg = x(:).';
pns = comm.PNSequence('Polynomial',[1 1 0 0 1 1 1 1 1], ...
    'InitialConditions', fliplr(msg(2:end)),'SamplesPerFrame',51);
y = [msg(1); xor(msg(1),pns())];
end

function y = num2bits(x,n,s)
y = int2bit(round(x./s),n);
end