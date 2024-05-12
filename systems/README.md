
# Systems

This systems module contains a collection of dynamic models used across various directories to demonstrate control systems. The library of the included modules is work in progress. The module also includes an abstract base class `basemodel.m` to allow development of custom models. Please see the src directory for the implementations and further details.

## Available Models

The repository includes several example models, each encapsulated in its class that inherits from a base model class `basemodel.m`. Below is a list of the currently available dynamic models:

- **Satellite2D**: A two-dimensional satellite model, which serves as an example of a double-integrator system.
- **Pendulum**: A simple point-mass pendulum model.
- **Cartpole**: A model of a cart with a pole, which serves as a simple example of under-actuated system.
- **RRArm**: A model of a plannar RR robot arm (double-pendulum system)

Each model is defined in its own MATLAB class file within the `/systems` directory of this repository.

## Model Methods

All dynamic models inherit from the `basemodel` class and implement several methods that define their dynamics and behavior:

- `openloop(obj, t, x, u)`: Calculates the system's response to the input `u`.
- `closedloop(obj, t, x)`: Evaluates the system's behavior when the input vector is zero, hence providing insights into its inherent dynamics.
- `energy(obj, t, x)`: Estimates the total energy of the system at any given state and time.
- `M(obj, t, x)`: Generates the mass matrix of the system.
- `h(obj, t, x)`: Returns a vector of non-conservative forces acting on the system
- `B(obj, t, x)`: Provides the input matrix, which maps control inputs to their effects on the system states.

Given \({\bf q}\) as the vector of the generalized coordinates of the system and \({\bf u}\) as the vector of inputs to the system, The open loop dynamics of the system is

\[
\ddot{\bf q = M}^{-1}\left( {\bf B\, u - h} \right).
\]

## Simulator

The `simulator` class provides tools to simulate and visualize the behavior of different models. It core functionalities are:

- `solve()`: Integrates the system equations over time using numerical solvers.
    - `sol = simulator.solve(___)` returns `sol` as a structure with the fields
        - `sol.t` the time steps of the solution
        - `sol.x` the state vector at each time step. `sol.x(k,:)` is the state vector at `sol.t(k)`.
        - `sol.u` is empty if `closedloop` model is simulated. Otherwise it includes the input vector to `opendynamics` at each time step of `sol.t`. Each row corresponds to the input vector corresponding to the row of `sol.t`.
    - `sol = simulator.solve()` simulates the `closedloop` dynamics of the system using the default `tspan = [0, 10]` and default initial state `x0` as defined in the corresponding model instantiation.
    - `sol = simulator.solve(tspan)` uses the tsapn provided as the input for the `ode45` solver to find the trajectory of the `closedloop` dynamics.
    - `sol = simulator.solve(tspan, x0)` simulates the `closedloop` system from the provided `x0`
    - `sol = simulator.solve(tspan, x0, ctrl)` solves the control system based on the input `ctrl`. The input `ctrl` must be a callable object (function) with the structure `u = ctrl(t,x)`.
- `plot()`: Visualizes the state trajectories and system inputs of the system in time.
    - `ax = simulator.plot(sol)` plots the trajectories of the system that are obtained by calling `solve` and returns `ax` as an array of of the handles to all the axes in the plot.
- `animate()`: Provides an animation of the model dynamics.
    - `simulator.animate(sol)` animates the system in a new figure and axes that is constructed based on class parameters and default values.
    - `simulator.animate(sol, ax)` uses `ax` as the animation axes.

Othe tools provided by the `simulator` are
- `simulator.eval(f, sol)` evaluates any function `f` in the form `f(t,x)` over every point of `sol.t` and `sol.x` and returens a matrix where each row is the output of `f` for the corresponding row of `sol.t` and `sol.x`.
- `simulator.make_plot_axes()` creates an axes to plot time based trajectories or function values using `simulator` defaults and returnes the handle to the axes object.
    - `ax = simulator.make_plot_axes` creates a new axes based on the `simulator` object instantiation deafults.
    - `ax = simulator.make_plot_axes(ax)` sets `ax` properties to be the default properties of the `simulator` object instantiation.
- `simulatormake_animation_axis()` creates an animation axes based on `simulator` defaults and returnes the handle to the axes object.
    - `ax = simulatormake_animation_axis` builds a new axes in a new figure.
    - `ax = simulatormake_animation_axis(ax)` updates the properties of `ax` based on `simulator` defaults.


## Examples

The repository includes examples that demonstrate how to use each model and show cases some of the simulator functionalities:

| Model           | Description                                  | File           |
|-----------------|----------------------------------------------|----------------|
| Satellite2D     | Closed-loop example for Satellite2D model    | example_01a.m  |
| Satellite2D     | A simple PD control of Satellite2D model     | example_01b.m  |
| Pendulum        | Simulation of a free swing of the pendulum   | example_02a.m  |
| Pendulum        | A PD controller fot the the pendulum system  | example_02a.m  |
| CartPole        | Free motion of the cartpole system           | example_03a.m  |
| RRArm           | Feedback linearization of the RRArm system   | example_04a.m  |


## Note
If you want to utilize the systems in other directories of this repository, make sure to add 
`addpath ../systems/src/`
to the script.


## Getting Started

To get started with these examples, clone the repository and navigate to the `/systems` directory:

```bash
git clone https://github.com/siamakfaal/control_examples.git
cd control_examples/systems
```

You can run each example MATLAB script directly in your MATLAB environment.

## Contributing

Contributions to expand or improve the examples in this repository are welcome! Please feel free to fork the repository, make changes, and submit a pull request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
