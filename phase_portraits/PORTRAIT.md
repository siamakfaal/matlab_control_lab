# PORTRAIT Class for Drawing Phase Portraits of Dynamical Systems

This class provides functionalities for visualizing the trajectories of systems of differential equations in phase space, offering customizable options for the appearance of both the vector field and the trajectories.

## Properties

- **canvas**: A matrix defining the dimensions of the phase space to visualize. Format: `[xmin, xstep, xmax; ymin, ystep, ymax]`. Default is `[-1, 1, 0.1; -1, 1, 0.1]`.
- **xticks**: A vector specifying the x-axis ticks for the phase portrait.
- **yticks**: A vector specifying the y-axis ticks for the phase portrait.
- **quiver_color_map**: Color map used for the quiver plot representing the vector field. Default is `'abyss'`.
- **trajectory_color**: RGB vector specifying the color of the trajectories. Default is `[0, 0.447, 0.741]`.
- **quiver_line_width**: Line width of the quiver plot vectors. Default is 2.
- **quiver_arrow_scale**: Scale of the arrowheads in the quiver plot. Default is 0.1.
- **quiver_max_headsize**: Maximum size of the arrowheads in the quiver plot. Default is 1.
- **trajectory_line_width**: Line width of the trajectories. Default is 1.
- **label_font_size**: Font size for the labels on the plot. Default is 25.

## Methods

- **portrait**: Constructor for the portrait class. Initializes a new instance with a specified canvas and optional parameters for customizing the appearance. Syntax: `obj = portrait(canvas, 'PropertyName', PropertyValue, ...)`.
- **icgrid**: Generates a grid of initial conditions based on the specified canvas dimensions or on specified boundaries according to a direction string. The direction string can contain the letters 'r' (right), 'l' (left), 'u' (up), 'd' (down), 'h' (horizontal), and 'v' (vertical), indicating where the initial conditions should be placed. `x0 = obj.icgrid()` generates a grid covering the entire canvas. `x0 = obj.icgrid(directionString)` where 'directionString' is an optional argument specifying the boundary directions for initial conditions.
- **draw**: Draws the phase portrait for a given system of differential equations using the specified initial conditions and over a given time interval. Syntax: `obj.draw(f, x0, tspan)` where 'f' is a function handle to the system of differential equations, 'x0' is a matrix of initial conditions, and 'tspan' is the time interval as a two-element vector `[tstart, tend]`.
- **trajectories**: Computes and plots the trajectories of the dynamical system for the given initial conditions over the specified time span. This method is typically called within the draw method. Syntax: `obj.trajectories(f, x0, tspan)`.
- **directions**: Generates a directional field (quiver plot) for the dynamical system on the specified canvas. This method is typically called within the draw method to overlay the directional field on the phase portrait. Syntax: `obj.directions(f)`.
- **buildaxes**: Constructs and formats the axes for the phase portrait according to the class properties such as xticks, yticks, and label_font_size. This method ensures the phase portrait is properly scaled and labeled. Syntax: `obj.buildaxes()`.
- **findequilibria**: Numerically finds the equilibrium points of the dynamical system within the specified canvas. Equilibrium points are those where the derivative (velocity) of the system is zero. Syntax: `eqPoints = obj.findequilibria(f)` where 'f' is the function handle to the system of differential equations, and 'eqPoints' is an array of points (coordinates) where the system is in equilibrium.

## Example

```matlab
% Define a dynamical system
f = @(t, x)[x(1)+x(2)+x(1)^2+x(2)^2; x(1)-x(2)-x(1)^2+x(2)^2];

% Initialize the phase portrait object
phaseplane = portrait([-2.5, 0.2, 1; -2.5, 0.2, 1]);

% Generate a grid of initial conditions on the right and upper boundaries
x0 = phaseplane.icgrid('ru');

% Find equilibrium points of the system
eqPoints = phaseplane.findequilibria(f);

% Draw the phase portrait
phaseplane.draw(f, x0, [0, 10]);
