% JBini.m

% Created by Arend L. Schwab and Jim Papadopoulos, 18-Jun-2003
% Copyright © 2003-2015 Schwab, Papadopoulos, Ruina, & Dressel
% Delft University of Technology & Cornell University

% Modification History
% 11/01/05 AED - new setting draw_root_lines
%              - New user parameter: vel_steps
% 11/03/05 AED - New user parameter: min_vel
% 11/06/05 AED - New constants XY_UV and min_uvcm_mag
%              - Change Swinn to Schwinn
% 11/08/05 AED - New bike configuration subdirectory
% 11/09/05 AED - Define plot colors here
%              - toggle between double root finding methods
% 11/10/05 AED - Define more colors and dot sizes
%              - Switch color designations to RGB
%              - keep name and version in single place: JBini.m', ...
%              - setting to save user settings, or not
% 11/11/05 AED - Define more colors, sizes, and types
%              - Option to use styles in plot4 window
% 11/15/05 AED - Option to display legend or not
%              - option to run in 'verbose' mode or not
% 10/11/06 AED - Option to shade stable region and pick color
%              - Set minimum eigenvalue for stability threshold
% 09/09/12 AED - Update version number
%-------------------------------------------------------------------
% constants
%-------------------------------------------------------------------

app_name = 'JBike6';
app_version ='February 8, 2015';

min_eig_val_stable_thresh = -1e-15;
%-------------------------------------------------------------------
% user modifiabale settings
%-------------------------------------------------------------------
save_settings = 0;            % 1 = save user settings to configuration file

gravity = 9.80665;            % gravity (meters/second/second)

min_vel = 0.0;
max_vel = 10.0;               % default maximum velocity to investigate
vel_steps = 101;

angle_units = 2;              % 1 = radians, 2 = degrees
draw_coord_axes = 1;          % 1 = yes, 0 = no
draw_axis_lines = 1;          % 1 = yes, 0 = no
draw_grid_lines = 1;          % 1 = yes, 0 = no
draw_additional_plots = 0;    % 1 = yes, 0 = no
use_styles_in_plot4 = 1;      % 1 = yes, 0 = no
draw_root_lines = 0;          % 1 = yes, 0 = no
draw_weave_speed = 1;         % 1 = yes, 0 = no
draw_capsize_speed = 1;       % 1 = yes, 0 = no
shade_stable_region = 1;      % 1 = yes, 0 = no
draw_imaginary_parts = 0;     % 1 = yes, 0 = no
draw_legend = 1;              % 1 = yes, 0 = no
XY_UV = 1;                    % 1 = UV, 0 = XY
min_uvcm_mag = 1e-6;          % minimum u or v of cm magnitude
work_dir = 'JBini_files';     % where all the bike configuration files are kept
double_root_method = 1;       % 1 = interpolate from eigenvalues, 0 = analytically
run_verbose = 1;              % 1 = yes, 0 = no

%-------------------------------------------------------------------
% styles
%-------------------------------------------------------------------

% options are:
%
%    [0 0 1]  blue          .  point              -     solid
%    [0 1 0]  green         o  circle             :     dotted
%    [1 0 0]  red           x  x-mark             -.    dashdot 
%    [0 1 1]  cyan          +  plus               --    dashed   
%    [1 0 1]  magenta       *  star             (none)  no line
%    [1 1 0]  yellow        s  square
%    [0 0 0]  black         d  diamond
%    [1 1 1]  white         v  triangle (down)
%                           ^  triangle (up)
%                           <  triangle (left)
%                           >  triangle (right)
%                           p  pentagram
%                           h  hexagram

real_eigen_color = [0 0 1]; % blue
imag_eigen_color = [0 1 1]; % cyan
weave_speed_color = [0 1 0]; % green
capsize_speed_color = [1 0 0]; % red
double_root_speed_color = [1 0 1]; % magenta
stable_region_shade_color = [.9 1 .9]; %light green
axis_color = [0 0 0];              % black
plot_background_color = [1 1 1];   % white

real_eigen_symbol = '.';
imag_eigen_symbol = '.';
weave_speed_line_type = '-';
capsize_speed_line_type = '-';
double_root_speeds_line_type = '-';

real_eigen_size = 5;
imag_eigen_size = 5;
weave_speed_size = 1;
capsize_speed_size = 1;
double_root_speeds_size = 1;

%-------------------------------------------------------------------
% Bike parameters
%-------------------------------------------------------------------
%leave this in so user can still run just JBike6.m without user interface
%these values get overwritten when JBini_*.m is called from JBike6GUI.m

%You may change values, but do not remove nor change variable names.
%Unused variables (front basket, for example) must be set to zero.
%All units are meters, kilograms, and radians

%Read the help file (JBhelp.htm) for more details about parameters and
%their limitations, sign conventions, units, and illustrations.

	%-------------------------------------------------------------------
	% Schwinn Crown 
	%-------------------------------------------------------------------
	% bike name
	name = 'steerbywire';
	% wheelbase is distance between the rear and front contact pnt
	wheelbase = 1.03; %meters
	% lambda is the angle between the steer-axis and the y-axis,
	% positive along the z-axis (ccw)
	lambda =  0.2967; %radians
	head_angle = pi/2-lambda;
	% trail is the distance from front contact pnt to the intersection of
	% the steer axis with the ground.
	trail = 0.0665;
	%rearwheel
	% Diameter of the rear wheel
	Drearwheel = 0.6858;
	% mass of the rear wheel
	mrearwheel = 8.5;
	% Moments of inertia of the rear wheel along the principal axis 1, 2 and z
	Irearwheel =  [0.095625 0.095625  0.19125];
	% alpha is angle between 1-axis and x-axis, positive along z-axis (ccw)
	alphaIrearwheel = 0;
	% frontwheel
	Dfrontwheel =  0.6858;
	mfrontwheel = 1.84;
	% Ifrontwheel = [I11 I22 Izz]
	Ifrontwheel = 0.8*mfrontwheel*Dfrontwheel^2/4.*[1/2 1/2 1];
	alphaIfrontwheel = 0;
	% rider
	mrider = 80;
	% the position of the cm is measure from the origin at the rear contact pnt
	xcmrider = 0.3;
	ycmrider = 1.2;
	% Irider = [I11 I22 Izz]
	Irider = [10.53112887414928 2.46887112585073 12];
	% alpha angle between 1-axis and x-axis, positive along z-axis (ccw)
	alphaIrider = -0.25957305712326;
	% frame
	mframe = 2.5;
	xcmframe = 0.4;
	ycmframe = 0.6;
	% Iframe = [I11 I22 Izz]
	Iframe = [0.05857864376269   0.34142135623731   0.4];
	alphaIframe = 0.39269908169872;
	% fork
	mfork = 1.5;
	% the cm of the fork is measured in the uv-coordinate system
	% uv is located at the intersection of the steer axis with the ground
	% and the axis are rotated over the angle lamda
	ucmfork = 0;
	vcmfork = Dfrontwheel;
	% Ifork = [I11 I22 Izz];
	Ifork = mfork*(Dfrontwheel)^2/12.*[1 0.1^2 1];
  Ifork(1:2:3)=0.096;
	% alpha angle between 1-axis and x-axis, positive along z-axis (ccw)
	alphaIfork = lambda;
	% basket
	mbasket = 0;
	ucmbasket = 0;
	vcmbasket = 0;
	% Ibasket = [I11 I22 Izz];
	Ibasket = [0.00 0.00 0.00];
	% alpha angle between 1-axis and x-axis, positive along z-axis (ccw)
	alphaIbasket = 0;
	% rack
	mrack = 4;
	xcmrack = 0.4;
	ycmrack = 0.55;
	% Irack = [I11 I22 Izz]
	Irack = [0.02 0.02 0.02];
	alphaIrack = 0;


