% Simscape(TM) Multibody(TM) version: 7.5

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(1).translation = [0.0 0.0 0.0];
smiData.RigidTransform(1).angle = 0.0;
smiData.RigidTransform(1).axis = [0.0 0.0 0.0];
smiData.RigidTransform(1).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [1.7357675057852155 3.1895826203284248 5.9900849212099754];  % in
smiData.RigidTransform(1).angle = 0;  % rad
smiData.RigidTransform(1).axis = [0 0 0];
smiData.RigidTransform(1).ID = "RootGround[Charging_Socket_CCS_Type2_V1-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(1).mass = 0.0;
smiData.Solid(1).CoM = [0.0 0.0 0.0];
smiData.Solid(1).MoI = [0.0 0.0 0.0];
smiData.Solid(1).PoI = [0.0 0.0 0.0];
smiData.Solid(1).color = [0.0 0.0 0.0];
smiData.Solid(1).opacity = 0.0;
smiData.Solid(1).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.69931542211320374;  % kg
smiData.Solid(1).CoM = [-3.7946337903588139e-05 -0.73414082412587678 -40.184145022820601];  % mm
smiData.Solid(1).MoI = [1349.5655507597069 1000.6925892078009 869.43152795165599];  % kg*mm^2
smiData.Solid(1).PoI = [31.167865043168479 4.4460058587505857e-05 -0.0043690802534155412];  % kg*mm^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Charging_Socket_CCS_Type2_V1*:*Default";

