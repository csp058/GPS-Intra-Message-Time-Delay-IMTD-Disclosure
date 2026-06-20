% =========================================================================
% IMTD (INTRA-MESSAGE TIME-DELAYED) GPS L1C BASEBAND SIMULATOR
% =========================================================================
% Description:
%   This script provides a comprehensive end-to-end baseband simulation of 
%   the IMTD architecture on the GPS L1C (CNAV-2) signal. Unlike previous 
%   methods that rely on physical-layer puncturing, this architecture embeds 
%   cryptographic parameters strictly within the boundaries of standard 
%   CNAV-2 data frames to eliminate inter-epoch dependencies and halve 
%   authentication latency.
%
% Key Features & Pipeline:
%   1. Cryptographic Initialization: 
%      - Generates an ECDSA-512 (secp256r1) key pair (preserved for future use).
%      - Constructs a 128-bit TESLA hash chain.
%   2. IMTD Signal Assembly (Tx):
%      - Generates pure GPS L1C data (L1Cd) and pilot (L1Cp) without puncturing.
%      - Assembles 1 Epoch consisting of 5 Frames (nav1 ~ nav5).
%      - nav1~nav4: Carry standard navigation data.
%      - nav5: Carries the IMTD payload in standard CNAV-2 LDPC/BCH format.
%          * Start of Message (SoM, Sub-frame 2): 128-bit MAC
%          * End of Message (EoM, Sub-frame 3): 128-bit Epoch Key (K_j)
%   3. Memory-Optimized Realistic Channel Modeling:
%      - Employs a symbol-by-symbol RF chunking mechanism to simulate a full 
%        90-second epoch without memory overflow.
%      - Applies AWGN (40 dB-Hz C/N0) and a residual Doppler error (5 Hz).
%   4. Signal Tracking (Rx):
%      - Implements a realistic Phase-Locked Loop (PLL) with an ideal DLL.
%   5. IMTD Data Demodulation & Parsing:
%      - Recovers CNAV-2 subframes via BCH and LDPC decoding iteratively.
%      - Parses the strictly intra-message MAC and Key from nav5.
%   6. End-to-End Cryptographic Authentication:
%      - Verifies the TESLA Epoch Key via hash chain.
%      - Authenticates the recovered navigation subframes using the MAC.
%
% Author : Chang-Seop Park, Dankook Univ.
% Date   : June 21, 2026.
% =========================================================================

%% --------------------------------------------------------
%% Part 1: Signal Generation & Cryptographic Initialization (IMTD CNAV-2 Format)
%% --------------------------------------------------------
clc; clear; close all;
fprintf('[Part 1] IMTD Signal Generation & Cryptographic Initialization (CNAV-2 Format)\n');

%% 1. ECDSA Key-Pair Generation (Preserved for future extensions)
fprintf('[Init] Generating ECDSA (secp256r1) Key Pair...\n');
keyGen = java.security.KeyPairGenerator.getInstance('EC');
ecSpec = java.security.spec.ECGenParameterSpec('secp256r1');
keyGen.initialize(ecSpec, java.security.SecureRandom());
keyPair = keyGen.generateKeyPair();
privateKey = keyPair.getPrivate();
publicKey = keyPair.getPublic();

PRNID = 7;                        
f_chip = 1.023e6; 
fs = f_chip * 24;                       
samples_per_chip = fs / f_chip;

%% 2. Baseband Code Generation
[L1Cd, L1Cp, L1Co] = gpsL1CCodes(PRNID);

num_frames_per_epoch  = 5;
num_symbols_per_frame = 1800;
num_symbols_warmup    = 200;
num_symbols_payload   = num_frames_per_epoch * num_symbols_per_frame; % 5 Frames (9000 symbols)
num_symbols_buffer    = 10; 
total_symbols         = num_symbols_warmup + num_symbols_payload + num_symbols_buffer;

%% 3. Pure Pilot Channel Generation (Puncturing completely removed)
% Replicate L1Co (length 1800) sufficiently to cover the extended total symbols (9210)
num_repeats = ceil(total_symbols / length(L1Co));
L1Co_extended = repmat(L1Co, num_repeats, 1); 
L1Co_extended = L1Co_extended(1:total_symbols); 

L1Co_expanded = repelem(L1Co_extended, 10230, 1);  
L1Cp_expanded = repmat(L1Cp, total_symbols, 1);

L1Co_sig = 1 - 2*double(L1Co_expanded);   
L1Cp_sig = 1 - 2*double(L1Cp_expanded);	               	    

% Combine pilot components purely for synchronization purposes (No data embedding)
combined_Psig = L1Cp_sig .* L1Co_sig; 
fprintf('[Tx] Pure Pilot Channel generated (No Puncturing).\n');

%% 4. CNAV-2 Data Generation (nav1 ~ nav4)
cfgCNAV2 = gpsNavigationConfig('SignalType', 'CNAV2', 'PRNID', PRNID);   
[dataCNAV2_full, rawSF1_full, rawSF2_full, rawSF3_full] = gpsNAVDataEncode(cfgCNAV2);

% Extract encoded symbols and raw data bits for nav1 to nav4
nav1_4_encoded_bits = dataCNAV2_full(1 : 4 * num_symbols_per_frame);
nav1_4_raw_bits = [rawSF1_full(:, 1:4); rawSF2_full(:, 1:4); rawSF3_full(:, 1:4)];

%% 5. Cryptographic Initialization & TESLA Hash Chain
fprintf('[Tx] Generating TESLA Hash Chain (n=10)...\n');
n_chain = 10;
K_chain = cell(n_chain + 1, 1);
K_chain{11} = randi([0 1], 128, 1); % [IMPORTANT] Must be strictly set to 128 bits!

for i = 10:-1:1
    K_chain{i} = compute_proxy_hash(K_chain{i+1}, 128); % [IMPORTANT] Must be strictly set to 128 bits!
end

root_key = K_chain{1};              % K_0
current_epoch_key = K_chain{5};     % K_4 (Current Epoch Key, 128 bits)

% ECDSA Signature Generation and Preservation (For future extensions)
ecdsaSign = java.security.Signature.getInstance('SHA256withECDSA');
ecdsaSign.initSign(privateKey);
k0_str = char(root_key' + '0');
k0_bytes = typecast(uint8(bin2dec(reshape(k0_str, 8, 16)')), 'int8');
ecdsaSign.update(k0_bytes);
derSignature = ecdsaSign.sign();
root_signature = extract_512bit_signature(derSignature);

%% 6. IMTD nav5 Frame Construction (Strict CNAV-2 Format)
fprintf('[Tx] Constructing nav5 with IMTD Payload (MAC at SoM, Key at EoM)...\n');

% Generate MAC (Bind nav1~nav4 data with the current epoch key)
mac_input = [nav1_4_raw_bits(:); current_epoch_key];
MAC_j = compute_proxy_hash(mac_input, 128); % 128 bits

% --- Sub-frame 1 (TOI / BCH Encoding) ---
nav5_toi = mod(cfgCNAV2.L1CTOI + 4, 400); % TOI value for nav5 (+4 offset from nav1)
toiBits = int2bit(nav5_toi, 9);
s1Bits_nav5 = custom_gpsTOIEnc(toiBits); % 52 bits

% --- Sub-frame 2 (MAC + Zeros / LDPC Encoding) ---
% Spec: 128-bit MAC || 472-bit 0's = 600 bits
sf2_raw_nav5 = [MAC_j; zeros(472, 1)]; 

load("L1CLDPCParityCheckMatrices.mat", "A1","B1","C1","E1","T1");
l1cLDPCA1 = logical(sparse(A1(:,1),A1(:,2),1,599,600));
l1cLDPCB1 = logical(sparse(B1(:,1),B1(:,2),1,599,1));
l1cLDPCC1 = logical(sparse(C1(:,1),C1(:,2),1,1,600));
l1cLDPCD1 = true;
l1cLDPCE1 = logical(sparse(E1(:,1),E1(:,2),1,1,599));
l1cLDPCT1 = logical(sparse(T1(:,1),T1(:,2),1,599,599));
cfgLDPCSubframe2 = ldpcEncoderConfig([l1cLDPCA1, l1cLDPCB1, l1cLDPCT1; l1cLDPCC1, l1cLDPCD1, l1cLDPCE1]);

s2Bits_nav5 = ldpcEncode(sf2_raw_nav5, cfgLDPCSubframe2); % 1200 bits

% --- Sub-frame 3 (Zeros + Key + CRC / LDPC Encoding) ---
% Spec: 122-bit 0's || 128-bit K_j = 250 bits

% [Safeguard 1] Enforce exact 128-bit column vector for the key
current_epoch_key = current_epoch_key(:); 
if length(current_epoch_key) ~= 128
    error('[Architecture Error] current_epoch_key length is not 128 bits! (Current: %d)', length(current_epoch_key));
end

sf3_data_nav5 = [zeros(122, 1); current_epoch_key]; 

crcGeneratorSubframe3 = comm.CRCGenerator('Polynomial', ...
    'z^24 + z^23 + z^18 + z^17 + z^14 + z^11 + z^10 + z^7 + z^6 + z^5 + z^4 + z^3 + z + 1');
sf3_raw_nav5 = crcGeneratorSubframe3(sf3_data_nav5); % 250 + 24 (CRC) = 274 bits

% [Safeguard 2] Final check for exact 274-bit length before LDPC encoding
if length(sf3_raw_nav5) ~= 274
    error('[Architecture Error] sf3_raw_nav5 length is not 274 bits! (Current: %d)', length(sf3_raw_nav5));
end

load("L1CLDPCParityCheckMatrices.mat", "A2","B2","C2","E2","T2");
l1cLDPCA2 = logical(sparse(A2(:,1),A2(:,2),1,273,274));
l1cLDPCB2 = logical(sparse(B2(:,1),B2(:,2),1,273,1));
l1cLDPCC2 = logical(sparse(C2(:,1),C2(:,2),1,1,274));
l1cLDPCD2 = true;
l1cLDPCE2 = logical(sparse(E2(:,1),E2(:,2),1,1,273));
l1cLDPCT2 = logical(sparse(T2(:,1),T2(:,2),1,273,273));
cfgLDPCSubframe3 = ldpcEncoderConfig([l1cLDPCA2, l1cLDPCB2, l1cLDPCT2; l1cLDPCC2, l1cLDPCD2, l1cLDPCE2]);

s3Bits_nav5 = ldpcEncode(sf3_raw_nav5, cfgLDPCSubframe3); % 548 bits

% --- Interleaving and nav5 Assembly ---
s2s3_nav5 = matintrlv([s2Bits_nav5; s3Bits_nav5], 38, 46);
nav5_encoded_bits = [s1Bits_nav5; s2s3_nav5]; % 1800 bits

% Integrate data payload for 1 complete Epoch (nav1 ~ nav5)
nav1_4_symbols = 1 - 2 * double(nav1_4_encoded_bits);
nav5_symbols   = 1 - 2 * double(nav5_encoded_bits);
epoch_payload_symbols = [nav1_4_symbols; nav5_symbols]; % Total 9000 Symbols

fprintf('[Tx] IMTD 1 Epoch (5 Frames, 9000 Symbols) successfully assembled in CNAV-2 format.\n');

%% 7. Baseband Chip Generation (Memory Optimized)
fprintf('[Tx] Generating Baseband Chips (Chunking Preparation)...\n');
% Maintain signals at the chip level to prevent memory overflow (Avoid full 90-sec RF upsampling here)

L1Cf = [ones(num_symbols_warmup, 1); epoch_payload_symbols; ones(num_symbols_buffer, 1)]; 
L1Cf_expanded = repelem(L1Cf, 10230, 1);
L1Cd_full = repmat(L1Cd, total_symbols, 1);  
combined_Dsig = L1Cf_expanded .* (1 - 2*double(L1Cd_full)); % Perfectly reconstructed data chips without double modulation

% Precompute fixed BOC subcarriers and patterns for 1 symbol (10ms) for high-speed processing
num_samples_sym = fs * 0.01;
t_sym = (0:num_samples_sym-1)' / fs;
sub_boc11_sym = sign(sin(2 * pi * 1.023e6 * t_sym));
sub_boc61_sym = sign(sin(2 * pi * 6.138e6 * t_sym));

boc61_pattern = false(10230, 1);
for i = 0:33:10230-33, boc61_pattern(i + [1, 5, 7, 30]) = true; end
is_boc61_sym = repelem(boc61_pattern, samples_per_chip, 1);

% Pure data code for demodulator correlation (excluding navigation bits)
D_code_up = repelem((1 - 2*double(L1Cd)), samples_per_chip, 1);
local_d_pure = D_code_up .* sub_boc11_sym;

f_if = 4e6; doppler_true = 2505; 
% *Assume ideal code_delay acquisition (0) to prevent chunking boundary leakage

%% --------------------------------------------------------
%% Part 2: Coarse Signal Acquisition (Using 1st Symbol)
%% --------------------------------------------------------
fprintf('[Part 2] Coarse Signal Acquisition (1st Symbol Only)\n');

% Temporarily generate RF signal for the 1st symbol only for Coarse Acquisition
D_up_acq = repelem(combined_Dsig(1:10230), samples_per_chip, 1);
P_up_acq = repelem(combined_Psig(1:10230), samples_per_chip, 1);

L1Cp_wave_acq = zeros(num_samples_sym, 1);
L1Cp_wave_acq(is_boc61_sym)  = P_up_acq(is_boc61_sym) .* sub_boc61_sym(is_boc61_sym);
L1Cp_wave_acq(~is_boc61_sym) = P_up_acq(~is_boc61_sym) .* sub_boc11_sym(~is_boc61_sym);
L1Cd_wave_acq = D_up_acq .* sub_boc11_sym;

L1C_total_acq = (0.5 * L1Cd_wave_acq) + ((sqrt(3)/2) * L1Cp_wave_acq);
L1C_tx_acq = L1C_total_acq .* cos(2 * pi * (f_if + doppler_true) * t_sym);
L1C_rx_acq = awgn(L1C_tx_acq, 40 - 10*log10(fs), 'measured');

acq_map = zeros(21, num_samples_sym);
doppler_search = -5000:500:5000;
for i = 1:length(doppler_search)
    carrier_i = exp(-1j * 2 * pi * (f_if + doppler_search(i)) * t_sym);
    acq_map(i, :) = abs(ifft(fft(L1C_rx_acq .* carrier_i) .* conj(fft(L1Cp_wave_acq)))).^2;
end
[~, max_idx] = max(acq_map(:));
[f_idx, ~] = ind2sub(size(acq_map), max_idx);

%% --------------------------------------------------------
%% Part 3: Signal Tracking (Chunked RF Generation & PLL)
%% --------------------------------------------------------
fprintf('[Part 3] Signal Tracking (Symbol-by-Symbol Chunking) \n');

zeta = 0.707; PLL_BW = 10;
C1_pll = 2 * zeta * (PLL_BW * 8/3) / (2*pi);
C2_pll = ((PLL_BW * 8/3)^2 * 0.01) / (2*pi);

curr_doppler = doppler_search(f_idx); 
rem_doppler_phase = 0; pll_I = 0;
accu_prompt = zeros(total_symbols, 1);
track_history = zeros(total_symbols, 1);

fprintf('Tracking %d symbols... \n', total_symbols);

% Prevent memory overflow: Iteratively generate -> track -> discard RF data symbol-by-symbol
for step = 1:total_symbols
    if mod(step, 1000) == 0, fprintf(' [%d / %d] processed...\n', step, total_symbols); end
    
    % --- 1. Dynamic RF Generation for CURRENT Symbol ---
    idx_s = (step-1)*10230 + 1;
    idx_e = step*10230;
    
    D_up = repelem(combined_Dsig(idx_s:idx_e), samples_per_chip, 1);
    P_up = repelem(combined_Psig(idx_s:idx_e), samples_per_chip, 1);
    
    L1Cp_wave = zeros(num_samples_sym, 1);
    L1Cp_wave(is_boc61_sym)  = P_up(is_boc61_sym) .* sub_boc61_sym(is_boc61_sym);
    L1Cp_wave(~is_boc61_sym) = P_up(~is_boc61_sym) .* sub_boc11_sym(~is_boc61_sym);
    L1Cd_wave = D_up .* sub_boc11_sym;
    
    L1C_total = (0.5 * L1Cd_wave) + ((sqrt(3)/2) * L1Cp_wave);
    
    % Absolute time for continuous phase
    t_chunk = ((step-1)*num_samples_sym : step*num_samples_sym - 1)' / fs;
    L1C_tx = L1C_total .* cos(2 * pi * (f_if + doppler_true) * t_chunk);
    L1C_rx_chunk = awgn(L1C_tx, 40 - 10*log10(fs), 'measured');
    
    % --- 2. PLL Wipeoff & Tracking ---
    phase_doppler = rem_doppler_phase + 2*pi*curr_doppler * t_sym;
    rem_doppler_phase = mod(rem_doppler_phase + 2*pi*curr_doppler * 0.01, 2*pi);
    
    sig_p = L1C_rx_chunk .* exp(-1j * (2*pi*f_if * t_chunk + phase_doppler));
    
    I_P = real(sum(sig_p .* L1Cp_wave)); 
    Q_P = imag(sum(sig_p .* L1Cp_wave));

    phi_err = atan2(Q_P, I_P);
    pll_I = pll_I + (C2_pll * phi_err); 
    curr_doppler = doppler_search(f_idx) + C1_pll * phi_err + pll_I;
    
    % Data correlator
    accu_prompt(step) = sum(sig_p .* local_d_pure);   
    track_history(step) = curr_doppler; 
end
fprintf(' Tracking Done!\n');

% BPSK Phase Alignment
valid_idx = num_symbols_warmup + 50 : total_symbols;
phase_offset = angle(mean(accu_prompt(valid_idx) .* sign(real(accu_prompt(valid_idx)))));
normalized_data = accu_prompt .* exp(-1j * phase_offset);

%% [Graph Output] Signal Tracking Performance (I/Q & Constellation)
figure('Name', 'Tracking Performance (1 Epoch)', 'Color', 'w', 'Position', [100, 100, 1000, 400]);

subplot(1, 2, 1);
plot(1:total_symbols, real(normalized_data), 'b', 'DisplayName', 'In-Phase (I)', 'LineWidth', 0.5); hold on;
plot(1:total_symbols, imag(normalized_data), 'r', 'DisplayName', 'Quadrature (Q)', 'LineWidth', 0.5);
xline(num_symbols_warmup, 'k--', 'Warm-up End', 'LineWidth', 2);
grid on; title('I/Q Correlator Outputs (5 Frames / 1 Epoch)');
xlabel('Symbol Index'); ylabel('Amplitude'); legend('Location', 'best');

subplot(1, 2, 2);
payload_complex = normalized_data(num_symbols_warmup+1 : end);
scatter(real(payload_complex), imag(payload_complex), 10, 'b', 'filled', 'MarkerFaceAlpha', 0.3);
grid on; hold on; xline(0, 'k-', 'LineWidth', 1.5); yline(0, 'k-', 'LineWidth', 1.5);
title('BPSK Constellation (Post Warm-up)'); xlabel('In-Phase (I)'); ylabel('Quadrature (Q)');
axis square; max_amp = max(abs(real(payload_complex))) * 1.5;
xlim([-max_amp, max_amp]); ylim([-max_amp, max_amp]);

%% --------------------------------------------------------
%% Part 4: CNAV-2 Epoch Demodulation (nav1 ~ nav5)
%% --------------------------------------------------------
fprintf('\n[Part 4] CNAV-2 Epoch Demodulation (nav1 ~ nav5)\n');

% Initialize LDPC matrices and CRC detector (Load once and reuse)
try
    load("L1CLDPCParityCheckMatrices.mat", "A1","B1","C1","E1","T1", "A2","B2","C2","E2","T2");
    H2 = [logical(sparse(A1(:,1),A1(:,2),1,599,600)), logical(sparse(B1(:,1),B1(:,2),1,599,1)), logical(sparse(T1(:,1),T1(:,2),1,599,599));
          logical(sparse(C1(:,1),C1(:,2),1,1,600)), true, logical(sparse(E1(:,1),E1(:,2),1,1,599))];
    H3 = [logical(sparse(A2(:,1),A2(:,2),1,273,274)), logical(sparse(B2(:,1),B2(:,2),1,273,1)), logical(sparse(T2(:,1),T2(:,2),1,273,273));
          logical(sparse(C2(:,1),C2(:,2),1,1,274)), true, logical(sparse(E2(:,1),E2(:,2),1,1,273))];
    crcDetector = comm.CRCDetector('Polynomial', 'z^24 + z^23 + z^18 + z^17 + z^14 + z^11 + z^10 + z^7 + z^6 + z^5 + z^4 + z^3 + z + 1');
catch
    error('[!] L1CLDPCParityCheckMatrices.mat not found. Please ensure it is in the path.');
end

nav1_4_recovered_bits = [];
rec_MAC = [];
rec_Key = [];

% Sequential decoding for 1 Epoch (5 Frames)
for frm = 1:5
    % Extract each frame (1800 symbols) separately
    start_idx = num_symbols_warmup + (frm-1)*1800 + 1;
    end_idx = num_symbols_warmup + frm*1800;
    frame_symbols = normalized_data(start_idx : end_idx);
    demod_bits = frame_symbols < 0;

    % 1) Sub-frame 1 (TOI / BCH Brute-force Recovery)
    errCnt = zeros(512, 1);
    for i = 0:511
        errCnt(i+1) = sum(xor(double(demod_bits(1:52)).', custom_gpsTOIEnc(int2bit(i, 9)).'));
    end
    [~, bestIdx] = min(errCnt);
    rec_toi_bits = int2bit(bestIdx - 1, 9);
    rec_sf1 = custom_gpsTOIEnc(rec_toi_bits);

    % 2) Sub-frames 2 & 3 (Deinterleaving for LDPC decoding)
    deintrlvd = matdeintrlv(demod_bits(53:1800), 38, 46);
    sf2LLR = (1 - 2 * double(deintrlvd(1:1200))) * 10;
    sf3LLR = (1 - 2 * double(deintrlvd(1201:1748))) * 10;

    % Perform LDPC Decoding
    decodedSF2 = ldpcDecode(sf2LLR, ldpcDecoderConfig(H2), 30);
    decodedSF3 = ldpcDecode(sf3LLR, ldpcDecoderConfig(H3), 30);

    if frm < 5
        % --- nav1 ~ nav4: Restore pure navigation data and verify CRC ---
        [~, err2] = crcDetector(double(decodedSF2));
        [~, err3] = crcDetector(double(decodedSF3));
        if err2 == 0 && err3 == 0
            fprintf('[Rx] nav%d: LDPC & CRC Passed.\n', frm);
        else
            fprintf('[!] nav%d: CRC Failed!\n', frm);
        end
        % Accumulate restored bitstreams for subsequent MAC verification
        nav1_4_recovered_bits = [nav1_4_recovered_bits; rec_sf1(:); double(decodedSF2(:)); double(decodedSF3(:))];
        
    else
        % --- nav5: IMTD Payload Extraction (Spec-based Parsing) ---
        fprintf('--------------------------------------------------\n');
        
        % Verify integrity of nav5's SF3 since it contains a CRC
        [~, err3] = crcDetector(double(decodedSF3));
        if err3 == 0
            fprintf('[Rx] nav5 Sub-frame 3: CRC Passed!\n');
        else
            fprintf('[!] nav5 Sub-frame 3: CRC Failed!\n');
        end

        % Spec 1. Extract the first 128 bits from Sub-frame 2 (600 bits) -> SoM: MAC
        rec_MAC = double(decodedSF2(1:128));   
        
        % Spec 2. Extract 128 bits after the 122-bit zero padding from Sub-frame 3 (274 bits) -> EoM: Key
        rec_Key = double(decodedSF3(123:250)); 
        
        fprintf('[Rx] nav5 successfully parsed (MAC and Key extracted).\n');
        fprintf('--------------------------------------------------\n');
    end
end

%% --------------------------------------------------------
%% Part 5: Cryptographic Verification (IMTD Architecture)
%% --------------------------------------------------------
fprintf('\n[Part 5] Cryptographic Verification (IMTD) \n');

% 1) Verify TESLA Key Chain (Hash(K_4) == K_3)
% *In a real environment, the receiver holds the previously verified K_3 in memory
stored_prev_epoch_key = K_chain{4}; % Receiver's Trust Anchor (K_3)
computed_prev_key = compute_proxy_hash(rec_Key, 128);
is_key_valid = isequal(computed_prev_key, stored_prev_epoch_key);

if is_key_valid
    fprintf('[Rx] 1. TESLA Key Chain Verify : PASS (Hash matched stored commitment K_3)\n');
else
    fprintf('[!] 1. TESLA Key Chain Verify : FAIL (Spoofing detected)\n');
end

% 2) Verify Navigation Message MAC (nav1 ~ nav4)
% MAC = Hash([nav1~nav4_bits || Key_j])
mac_input = [nav1_4_recovered_bits; rec_Key];
computed_mac = compute_proxy_hash(mac_input, 128);
is_mac_valid = isequal(computed_mac, rec_MAC);

if is_mac_valid
    fprintf('[Rx] 2. Navigation MAC Verify  : PASS (nav1~nav4 are Authentic & Intact)\n');
else
    fprintf('[!] 2. Navigation MAC Verify  : FAIL (Navigation Data Manipulation Detected!)\n');
end

% 3) Verify ECDSA Root Signature (Preserved logic for future extensions)
% *In this simulation, we verify the derived K_0 using the root_signature generated by the transmitter
reconstructed_der = repack_to_der(root_signature);
ecdsaVerify = java.security.Signature.getInstance('SHA256withECDSA');
ecdsaVerify.initVerify(publicKey);

% Reverse the hash chain from the received K_4 to recover K_0
derived_k = rec_Key;
for i = 1:4
    derived_k = compute_proxy_hash(derived_k, 128);
end
k0_str = char(derived_k' + '0');
k0_bytes = typecast(uint8(bin2dec(reshape(k0_str, 8, 16)')), 'int8');

ecdsaVerify.update(k0_bytes);
is_sig_valid = ecdsaVerify.verify(reconstructed_der);

if is_sig_valid
    fprintf('[Rx] 3. Root Signature Verify  : PASS (ECDSA-512 Validates Derived K_0)\n');
else
    fprintf('[!] 3. Root Signature Verify  : FAIL (Invalid Digital Signature for Root Key)\n');
end

% === Output Final Authentication Result ===
if is_key_valid && is_mac_valid && is_sig_valid
    fprintf('\n[+++] IMTD Cryptographic Authentication Fully Successful! Epoch is Genuine. [+++]\n');
end

%% --------------------------------------------------------
%% Local Helper Functions
%% --------------------------------------------------------

function hash_bits = compute_proxy_hash(input_bits, output_length)
    pad_len = 8 - mod(length(input_bits), 8);
    if pad_len ~= 8, input_bits = [input_bits; zeros(pad_len, 1)]; end
    
    num_bytes = length(input_bits) / 8;
    input_bytes = zeros(num_bytes, 1, 'uint8');
    for i = 1:num_bytes
        byte_bits = input_bits((i-1)*8 + 1 : i*8);
        input_bytes(i) = uint8(bin2dec(char(byte_bits' + '0')));
    end

    md = java.security.MessageDigest.getInstance('SHA-256');
    hash_bytes = typecast(md.digest(int8(input_bytes)), 'uint8');

    hash_bits_str = dec2bin(hash_bytes, 8)';
    hash_bits_all = hash_bits_str(:) - '0';
    hash_bits = hash_bits_all(1:output_length);
end

function y = custom_gpsTOIEnc(x)
    msg = x(:).';
    pns = comm.PNSequence('Polynomial',[1 1 0 0 1 1 1 1 1], ...
        'InitialConditions', fliplr(msg(2:end)),'SamplesPerFrame',51);
    y = [msg(1); xor(msg(1),pns())];
end

function out = trim_or_pad(val, targetLen)
    if length(val) > targetLen
        out = val(end-targetLen+1:end);
    else
        out = [zeros(targetLen-length(val), 1, 'uint8'); val(:)];
    end
end

function derBytes = repack_to_der(sigBits)
    sig_str = char(sigBits' + '0');
    raw64 = uint8(bin2dec(reshape(sig_str, 8, 64)'));   

    r_raw = raw64(1:32);
    s_raw = raw64(33:64);    
    r = format_der_integer(r_raw);
    s = format_der_integer(s_raw);
    rLen = length(r);
    sLen = length(s);
    totalLen = 4 + rLen + sLen;
   
    der = [uint8(48); uint8(totalLen);
           uint8(2); uint8(rLen); r(:);
           uint8(2); uint8(sLen); s(:)];
           
    derBytes = typecast(der, 'int8');
end

function out = format_der_integer(val)
    idx = 1;
    while idx < length(val) && val(idx) == 0
        idx = idx + 1;
    end
    val = val(idx:end);
    if bitand(val(1), 128)
        out = [uint8(0); val(:)];
    else
        out = val(:);
    end
end

% --- ECDSA-512 Helper Functions ---
function sigBits = extract_512bit_signature(derBytes)
    der = typecast(derBytes, 'uint8');
    idx = 3; 
    idx = idx + 1; 
    rLen = double(der(idx)); idx = idx + 1;
    r = der(idx : idx+rLen-1); idx = idx + rLen;
    idx = idx + 1; 
    sLen = double(der(idx));
    idx = idx + 1;
    s = der(idx : idx+sLen-1);
    
    r = trim_or_pad(r, 32);
    s = trim_or_pad(s, 32);
    raw64 = [r(:); s(:)];
    bits_str = dec2bin(raw64, 8)';
    sigBits = (bits_str(:) - '0');
end 
