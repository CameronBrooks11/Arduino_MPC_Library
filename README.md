# Arduino Model Predictive Control (MPC) Library

This repository contains a comprehensive collection of Model Predictive Control (MPC) implementations for Arduino/Teensy systems, including both unconstrained and constrained versions. The library is designed to be compact and suitable for real-time embedded systems, avoiding dependencies on large libraries like Eigen or the C++ Standard Library.

Extension of the works found here:
- [pronenewbits/Arduino_Constrained_MPC_Library](https://github.com/pronenewbits/Arduino_Constrained_MPC_Library)
- [pronenewbits/Arduino_Unconstrained_MPC_Library](https://github.com/pronenewbits/Arduino_Unconstrained_MPC_Library)

## Table of Contents
- [Background](#background)
- [Implementations](#implementations)
  - [Unconstrained MPC](#unconstrained-mpc)
  - [Constrained MPC](#constrained-mpc)
- [How to Use](#how-to-use)
- [Benchmark Results](#benchmark-results)
- [Closing Remarks](#closing-remarks)

## Background

### Unconstrained MPC
The unconstrained version of the MPC library aims to provide a clear and educational implementation of linear MPC. It is designed to be understandable for undergraduate control system engineering students while still being capable of real-time control system implementation. The library avoids dynamic memory allocation, making it suitable for mission-critical applications.

### Constrained MPC
The constrained MPC version extends the unconstrained implementation to handle constraints using Quadratic Programming (QP). It includes continuous constraints and mixed-integer constraints, making it suitable for more complex control scenarios.

## Implementations

### Unconstrained MPC
The unconstrained MPC library includes three main implementations:
1. **Naive Implementation:** A direct implementation of the MPC algorithm.
2. **Optimized Implementation:** An optimized version that precomputes constant matrices.
3. **Numerically Robust Implementation:** A least-squares reformulation for improved numerical stability.

#### Folders:
- `mpc_unconstrained`
- `mpc_unconstrained_opt`
- `mpc_unconstrained_lsq`

### Constrained MPC
The constrained MPC library provides two main implementations:
1. **Naive Implementation:** Direct implementation using Active Set methods.
2. **Optimized Implementation:** Uses Schur complement to solve the KKT system efficiently.

#### Folders:
- `mpc_constrained`
- `mpc_constrained_opt`

## How to Use

1. **Select the Implementation:**
  - **Unconstrained MPC:**
    - `mpc_unconstrained`: Naive implementation.
      - **Use when:** You are learning MPC concepts for the first time and need clear, readable code.
    - `mpc_unconstrained_opt`: Optimized implementation.
      - **Use when:** You need the fastest execution time for real-time applications.
    - `mpc_unconstrained_lsq`: Numerically robust implementation.
      - **Use when:** You need the most stable and reliable solution for systems prone to numerical issues.

  - **Constrained MPC:**
    - `mpc_constrained`: Naive implementation using Active Set methods.
      - **Use when:** You are implementing constrained MPC for educational purposes and need a straightforward approach.
    - `mpc_constrained_opt`: Optimized implementation using Schur complement.
      - **Use when:** You require efficient computation for complex systems with constraints in real-time environments.

2. **Configure the System:**
   - Modify `konfig.h` to set the length of `X, U, Z` vectors and sampling time `dt`.
   - Set MPC parameters like `Hp` (Prediction Horizon) and `Hu` (Control Horizon).

3. **Define the System:**
   - In the `.ino` file, define the linear system matrices `A, B, C` and initialization values `weightQ, weightR`.

4. **Initialize and Run:**
   - Initialize the MPC class and set non-zero initialization matrices with `MPC::vReInit(A, B, C, weightQ, weightR)`.
   - At every sampling time, call `MPC::bUpdate(SP, x, u)` to calculate the control value `u(k)`.

5. **Constructing a Model:**
   - To use MPC for controlling an application, first construct a mathematical model of your system. This typically involves deriving the state-space representation:

     `x(k+1) = A * x(k) + B * u(k)`

     `y(k) = C * x(k)`

   - Here, `x(k)` is the state vector, `u(k)` is the control input vector, and `y(k)` is the output vector. Matrix `A` describes the system dynamics, `B` maps the control input to the state changes, and `C` maps the state to the output.

6. **Translating to Variables:**
   - Once you have the matrices `A`, `B`, and `C`, define them in the `.ino` file. Set the control and prediction horizons (`Hu`, `Hp`) based on how far ahead you need to control and predict.
   - Define the weights `weightQ` and `weightR` to balance between tracking the reference and minimizing control effort.


### Notes:
- For Arduino configuration, set `SYSTEM_IMPLEMENTATION` to `SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO` in `konfig.h`.
- For PC configuration, set `SYSTEM_IMPLEMENTATION` to `SYSTEM_IMPLEMENTATION_PC` in `konfig.h`.

## Benchmark Results
Performed using a Teensy 4.0.

### Unconstrained MPC
- Naive implementation: 245 us (single precision) / 408 us (double precision)
- Optimized implementation: 37 us (single precision) / 65 us (double precision)
- Numerically robust implementation: 195 us (single precision) / 339 us (double precision)

### Constrained MPC
- No-constraint: 169 us (naive) / 35 us (optimized) / 11 us (optimized without bounds checking)
- Slew rate constraints: 556 us (naive) / 170 us (optimized) / 67 us (optimized without bounds checking)
- Slew rate + output constraints: 886 us (naive) / 327 us (optimized) / 127 us (optimized without bounds checking)

## Closing Remarks

I hope this library proves useful for your projects. Feel free to test, validate, and provide feedback. The code is published under the CC0 license, effectively placing it in the public domain. If you use this library, I would love to hear about your projects and applications, as it provides motivation to continue expanding this work.

For detailed documentation and additional resources, please refer to the individual folders and their respective README files.

Happy coding!
