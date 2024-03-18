# Phase Portrait Illustrator

This repository contains MATLAB files for drawing phase portraits of dynamical systems with two dimensional state space. The main functionality is encapsulated in the `portrait` class, which allows for the visualization of trajectories in a system's phase space.

## Files

- `portrait.m`: The MATLAB class providing the core functionality for drawing phase portraits.
- `example_01.m`: An example script demonstrating how to use the `portrait` class to visualize a dynamical system.

## Features

The `portrait` class offers several functionalities and properties for customizing the appearance of phase portraits:

- **Canvas Specification**: Allows for setting the dimensions of the phase space to be visualized.
- **Customizable Appearance**: Options to customize colors, line widths, and font sizes for different elements of the portrait, including trajectories and vector fields.
- **Quiver Plot Customization**: Includes settings for the color map, line width, arrow scale, and maximum head size of the quiver plot that represents the vector field.
- **Trajectory Customization**: Options to set the color and line width of trajectories.

## Example Usage

The provided example, `example_01.m`, demonstrates how to use the `portrait` class to draw a phase portrait for a given system of differential equations. The script outlines steps for setting up the system equations, initializing the `portrait` object with desired dimensions, generating initial conditions, and finally, drawing the phase portrait.

To run the example, ensure you have MATLAB installed and both `portrait.m` and `example_01.m` are in your current working directory. Then, execute `example_01.m` in MATLAB.

### Code Snippet

```matlab
clc; clear; close all;

f = @(t, x)[x(1)+x(2)+x(1)^2+x(2)^2; x(1)-x(2)-x(1)^2+x(2)^2];

phaseplane = portrait([-2.5, 0.2 ,1 ; -2.5, 0.2, 1]);
x0 = phaseplane.icgrid();
phaseplane.draw(f,x0,[0,10]);
