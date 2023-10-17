% Simscape(TM) Multibody(TM) version: 7.5

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(11).translation = [0.0 0.0 0.0];
smiData.RigidTransform(11).angle = 0.0;
smiData.RigidTransform(11).axis = [0.0 0.0 0.0];
smiData.RigidTransform(11).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [251.8313167688917 307.64298965187521 149.74861480966538];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(1).ID = "B[Link 2-1:-:rotor-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-1.2103390661855283e-13 0 2.8116693833810577e-13];  % mm
smiData.RigidTransform(2).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(2).axis = [0.57735026918962584 -0.57735026918962573 0.57735026918962573];
smiData.RigidTransform(2).ID = "F[Link 2-1:-:rotor-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [206.83131676889178 307.61143377069487 624.7091405035718];  % mm
smiData.RigidTransform(3).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(3).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(3).ID = "B[Link 2-1:-:Link1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-98.76523214981674 212.66673439229851 519.87428743748933];  % mm
smiData.RigidTransform(4).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(4).axis = [1 4.9987046203551211e-33 3.2543755198072419e-17];
smiData.RigidTransform(4).ID = "F[Link 2-1:-:Link1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [0 85.453869171747016 -41.324461755940177];  % mm
smiData.RigidTransform(5).angle = 0;  % rad
smiData.RigidTransform(5).axis = [0 0 0];
smiData.RigidTransform(5).ID = "B[rotor-1:-:Charger end2-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-18.062775075081248 50.378048610856965 136.81176939297779];  % mm
smiData.RigidTransform(6).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(6).axis = [1 6.5352630955804125e-33 3.464438572754793e-16];
smiData.RigidTransform(6).ID = "F[rotor-1:-:Charger end2-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [20.465798108060817 82.888728635938989 60.884055556143593];  % mm
smiData.RigidTransform(7).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(7).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(7).ID = "B[Mount assem-1:-:stand (1)-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [66.835823375065146 1209.1574814035037 379.00568415406974];  % mm
smiData.RigidTransform(8).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(8).axis = [-1.6653345369377351e-16 -0.70710678118654757 -0.70710678118654757];
smiData.RigidTransform(8).ID = "F[Mount assem-1:-:stand (1)-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [401.15865985260393 419.05362158515567 499.87428743748899];  % mm
smiData.RigidTransform(9).angle = 0;  % rad
smiData.RigidTransform(9).axis = [0 0 0];
smiData.RigidTransform(9).ID = "B[Link1-1:-:Mount assem-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [129.46579810806162 224.88872863593895 103.38405555614344];  % mm
smiData.RigidTransform(10).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(10).axis = [0.57735026918962562 -0.57735026918962595 0.57735026918962562];
smiData.RigidTransform(10).ID = "F[Link1-1:-:Mount assem-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [121.9450217346462 -716.64328133398794 21.44002682064783];  % mm
smiData.RigidTransform(11).angle = 0;  % rad
smiData.RigidTransform(11).axis = [0 0 0];
smiData.RigidTransform(11).ID = "RootGround[stand (1)-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(6).mass = 0.0;
smiData.Solid(6).CoM = [0.0 0.0 0.0];
smiData.Solid(6).MoI = [0.0 0.0 0.0];
smiData.Solid(6).PoI = [0.0 0.0 0.0];
smiData.Solid(6).color = [0.0 0.0 0.0];
smiData.Solid(6).opacity = 0.0;
smiData.Solid(6).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 12.892592802757415;  % kg
smiData.Solid(1).CoM = [0.95775551690735161 25.481662661328393 10.510554821912184];  % in
smiData.Solid(1).MoI = [10764.277258358985 306.29509493993481 10764.297995050054];  % kg*in^2
smiData.Solid(1).PoI = [-0.0045649813496589659 -0.00022145736621592685 0.20271357133450699];  % kg*in^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "stand (1)*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 2.9208292052673372;  % kg
smiData.Solid(2).CoM = [8.2511470949213344 12.111124356023344 16.492829435390117];  % in
smiData.Solid(2).MoI = [98.482665672720671 97.630948187603536 2.0172029628021413];  % kg*in^2
smiData.Solid(2).PoI = [0.0014982925273909779 0.89950724474774924 -0.00043763652095285825];  % kg*in^2
smiData.Solid(2).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Link 2*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 1.699268525060168;  % kg
smiData.Solid(3).CoM = [3.4327307137570005 6.1410907287794823 4.0688491001787241];  % in
smiData.Solid(3).MoI = [5.1938658382938838 11.073536520524952 12.357581797805864];  % kg*in^2
smiData.Solid(3).PoI = [-0.012484850292509288 0.012172810028573695 2.5362312352165803];  % kg*in^2
smiData.Solid(3).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "Mount assem*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 3.6820430067063881;  % kg
smiData.Solid(4).CoM = [7.8678129234584357 13.212635245928922 18.374841919820746];  % in
smiData.Solid(4).MoI = [28.442018111026982 150.87534410191665 176.68160041161775];  % kg*in^2
smiData.Solid(4).PoI = [-1.3764919000095162 -3.3126389488783805 -61.248932296868603];  % kg*in^2
smiData.Solid(4).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Link1*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.22687960300547713;  % kg
smiData.Solid(5).CoM = [0 31.921511687441313 0];  % mm
smiData.Solid(5).MoI = [169.14523687140067 162.60952569373174 169.2374875798283];  % kg*mm^2
smiData.Solid(5).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(5).color = [1 1 1];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = "rotor*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 0.58612192283777731;  % kg
smiData.Solid(6).CoM = [1.0031506577107432 5.0992473204526085 3.7702933304530766];  % in
smiData.Solid(6).MoI = [1.5287057362224146 2.3093261604598534 3.3988230635439067];  % kg*in^2
smiData.Solid(6).PoI = [-0.0042162410033328122 0.00076717069724215406 0.47457609950797286];  % kg*in^2
smiData.Solid(6).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = "Charger end2*:*Default";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the PrismaticJoint structure array by filling in null values.
smiData.PrismaticJoint(1).Pz.Pos = 0.0;
smiData.PrismaticJoint(1).ID = "";

smiData.PrismaticJoint(1).Pz.Pos = 0;  % m
smiData.PrismaticJoint(1).ID = "[Mount assem-1:-:stand (1)-1]";


%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(4).Rz.Pos = 0.0;
smiData.RevoluteJoint(4).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = 0.029091003667986712;  % deg
smiData.RevoluteJoint(1).ID = "[Link 2-1:-:rotor-1]";

smiData.RevoluteJoint(2).Rz.Pos = -66.074835044132726;  % deg
smiData.RevoluteJoint(2).ID = "[Link 2-1:-:Link1-1]";

smiData.RevoluteJoint(3).Rz.Pos = 172.79980935194521;  % deg
smiData.RevoluteJoint(3).ID = "[rotor-1:-:Charger end2-2]";

smiData.RevoluteJoint(4).Rz.Pos = -157.5929399664424;  % deg
smiData.RevoluteJoint(4).ID = "[Link1-1:-:Mount assem-1]";

