# Dynamics Linearization Investigation

This repository is dedicated to the exploration of how linearization affects the dynamics of systems. It contains scripts and tools for generating phase portraits of nonlinear systems and their linear approximations. The focus is on understanding the changes in system behavior when transitioning from a nonlinear to a linear model.

## Overview

The repository includes MATLAB scripts for computing and visualizing the dynamics of both nonlinear systems and their linear approximations. The primary script utilizes symbolic mathematics to calculate the Jacobian matrix at a point of interest (typically the origin) and uses this matrix to approximate the system's behavior near that point.

## Contents

- `linearization_script.m`: The main script that performs the analysis. It calculates the dynamics of a given nonlinear system, linearizes it around the origin, and plots the phase portraits of both the original system and its linear approximation.

## Prerequisites

- MATLAB
- Symbolic Math Toolbox
- Custom `phase_portraits` library (assumed to be located in a relative path from the script)

## Getting Started

1. Ensure MATLAB and the Symbolic Math Toolbox are installed on your system.
2. Clone this repository to your local machine.
3. Place the `phase_portraits` library in the correct relative path as indicated in the script (`../phase_portraits`).
4. Open MATLAB and navigate to the directory containing the script.
5. Run `linearization_script.m` to perform the analysis and generate the phase portraits.

## Script Details

The main script `linearization_script.m` performs several key operations:

- **System Definition**: Defines the nonlinear system of interest.
- **Linearization**: Calculates the linear approximation of the system at the origin by evaluating the Jacobian matrix.
- **Phase Portrait Generation**: Utilizes the `phase_portraits` library to generate and plot the phase portraits of both the nonlinear system and its linear approximation.

## Contributing

We welcome contributions to improve the analysis and expand the repository. Please feel free to fork the repository, make your changes, and submit a pull request.

## License

This project is open source and available under the [MIT License](LICENSE).