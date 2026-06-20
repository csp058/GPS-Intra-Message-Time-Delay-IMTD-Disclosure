# GPS-Intra-Message-Time-Delay-Disclosure
Code repository for IMTD for GPS L1C and CNAV-2 
## 📌 Overview
This repository contains a comprehensive, end-to-end MATLAB baseband simulator 
for the **Intra-Message Time-Delayed (IMTD)** architecture on the GPS L1C (CNAV-2) signal. 

Unlike conventional physical-layer puncturing methods, the IMTD architecture embeds 
cryptographic parameters strictly within the boundaries of standard CNAV-2 data frames. 
This effectively eliminates inter-epoch dependencies and strictly halves both the 
authentication latency and the receiver's memory overhead, providing a highly resilient 
and backward-compatible GNSS security paradigm.

## ✨ Key Features
1. **Cryptographic Initialization**
   - Generates an ECDSA-512 (secp256r1) key pair.
   - Constructs a 128-bit TESLA hash chain for delayed key disclosure.
2. **IMTD Signal Assembly (Strict CNAV-2 Format)**
   - Assembles 1 Epoch consisting of 5 Frames (9,000 symbols).
   - **nav1 ~ nav4:** Carries standard unmanipulated navigation data.
   - **nav5:** Carries the IMTD payload in standard CNAV-2 LDPC/BCH format without
     physical-layer pilot puncturing.
     - *Start of Message (SoM):* 128-bit MAC
     - *End of Message (EoM):* 128-bit Epoch Key ($K_j$)
3. **Memory-Optimized Realistic Channel Modeling**
   - Employs a dynamic **symbol-by-symbol RF chunking mechanism** to simulate
     a full 90-second epoch without RAM overflow.
   - Applies AWGN (40 dB-Hz C/N0) and a residual Doppler error (5 Hz).
4. **Signal Tracking (Rx)**
   - Implements a realistic Phase-Locked Loop (PLL) coupled with an ideal DLL
     to perfectly isolate and evaluate phase pull-in performance.
5. **Data Demodulation & Parsing**
   - Iterative CNAV-2 subframe recovery via BCH and LDPC decoding.
   - Parses the strictly intra-message MAC and Key strictly from the `nav5` frame.
6. **End-to-End Cryptographic Authentication**
   - Verifies the TESLA Epoch Key via the hash chain.
   - Authenticates the recovered raw navigation subframes using the MAC.
   - Validates the derived root key using the ECDSA-512 Signature.

## 📂 File Structure
* `Main_IMTD.m` : The main script to run the end-to-end simulation.
* `gpsL1CCodes.m` : Helper function generating GPS L1C baseband codes (Data, Pilot, Overlay).
* `gpsNavigationConfig.m` & `gpsNAVDataEncode.m` : Standard CNAV-2 data encoding helpers.
* `L1CLDPCParityCheckMatrices.mat` : Required parity-check matrices for CNAV-2 LDPC encoding/decoding.

> **Note:** Ensure all helper functions and the `.mat` file are in the same directory before running the main script.
