# QR Decomposition
## Final Project from Computer-aided VLSI System Design
### Overview
- **Objective:** Implement a simple MIMO receiver to demodulate RX data in a 5G MIMO system using QR decomposition.
- **Components:**
  - **TX Part:** Transmitted signal \( \mathbf{s} \)
  - **RX Part:** Received signal \( \mathbf{y} \), channel matrix \( \mathbf{H} \), noise \( \mathbf{n} \)
- **Goal:** Decode the received data by reversing the encoding process using QR decomposition.

### System Model
- **Equation:** \( \mathbf{y} = \mathbf{H}\mathbf{s} + \mathbf{n} \)
- **Channel:**
  - 4x4 matrix, complex numbers
  - Normal distribution random matrix \( \mathbf{H} \sim N(0, 1/4) \)
  - AWGN (Additive White Gaussian Noise) \( \mathbf{n} \sim N(0, 1) \)
- **Demodulation:** Use QR decomposition and Maximum Likelihood (ML) demodulation.
  - QR Decomposition: \( \mathbf{y} = \mathbf{Q}\mathbf{R}\mathbf{s} + \mathbf{n} \)
  - ML Demodulation: \( \hat{\mathbf{s}} = \arg\min_{\mathbf{s} \in \mathcal{A}} \|\mathbf{y} - \mathbf{H}\mathbf{s}\|^2 \)

### QR Decomposition
- **Purpose:** Reduce the complexity of ML demodulation.
- **Process:** Modified Gram-Schmidt Procedure
  - Orthogonalize matrix \( \mathbf{H} \)
  - Normalize vectors to create orthogonal matrix \( \mathbf{Q} \) and upper triangular matrix \( \mathbf{R} \)

---
### Algorithm
- **Traditional Gram-Schmidt Algorithm:** Susceptible to numerical errors.
- **Modified Gram-Schmidt Algorithm:** Used to reduce numerical errors introduced by traditional methods.
- We choose modified Gram-Schmidt Algorithm.

### Hardware Implementation
1. **Finite State Machine (FSM):** Controls the operation flow.
2. **Block Diagram:** Includes registers, pipeline stages, and data paths.
3. **Algorithm Steps:**
   - Read 200 data points into SRAM.
   - Compute Euclidean Distance for iteration 0 using inner product module.
   - Normalize vectors and store results in register files.
   - Perform inner products and orthogonalization in subsequent iterations.
   - Compute and store \( Y_{\hat{}} \) for each iteration.
   - Output final \( Y_{\hat{}} \) and \( R \) values.

### SRAM Configuration
- **Four SRAMs:** Two for real parts, two for imaginary parts.
- **Data Storage Format:** S3.12 for input, S3.16 for \( Y_{\hat{}} \) results.

### Designware Modules
- **dw_sqrt_pipe:** Used for square root operations in Euclidean Distance calculation.
- **dw_div_pipe:** Used for normalization division operations.
- **dw02_mult_2_stage:** Used for complex multiplications in inner product calculations.

### Fixed-Point (FXP) Settings
- **Input Data:** Converted to S3.12 format.
- **Square Root Input/Output:** S7.18 input, S3.10 output, extended to S3.16.
- **Normalization:** Performed with adjusted bit widths to balance error rate and area.

### Hardware Scheduling
- **Rounding vs. Truncation:** Most operations use truncation to simplify design.
- **Pipeline Optimization:** Utilizes pipeline stages for efficient data processing and minimizes idle states.

### Improvements and Techniques
- **Clock Gating:** Implemented manually to avoid setup/hold issues with multipliers.
- **Bit Width Optimization:** Reduces area and power consumption while maintaining accuracy.
- **Pipeline Stages:** Adjusted for optimal performance in square root and division operations.

### Final Performance
- **Area:** 336312.65 units
- **Power:** 24.3 units
- **Latency:** 1215205 cycles
