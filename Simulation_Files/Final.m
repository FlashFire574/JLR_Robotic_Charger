%% LINK LENGTHS
L1 = 0.54;
L2 = 0.475;

%% SYMBOLIC EXPRESSIONS
% THE EQUATIONS GOVERNING X&Y COORDS OF LINK2 FOR JOINT ANG
syms L_1 L_2 theta_1 theta_2 XE YE
XE_RHS = L_1*cos(theta_1) + L_2*cos(theta_1+theta_2);
YE_RHS = L_1*sin(theta_1) + L_2*sin(theta_1+theta_2);

XE_EQ = XE == XE_RHS;
YE_EQ = YE == YE_RHS;

%% EXPRESSION SOLVING & CONVERSION TO MATLAB FUNCTION
S = solve([XE_EQ YE_EQ], [theta_1 theta_2]);

TH1_MLF{1} = matlabFunction(S.theta_1(1),'Vars',[L_1 L_2 XE YE]);
TH1_MLF{2} = matlabFunction(S.theta_1(2),'Vars',[L_1 L_2 XE YE]);
TH2_MLF{1} = matlabFunction(S.theta_2(1),'Vars',[L_1 L_2 XE YE]);
TH2_MLF{2} = matlabFunction(S.theta_2(2),'Vars',[L_1 L_2 XE YE]);

%% GLOSSARY

% chapp ==> CHANGE IN APPROACH
% right_app ==> RIGHT APPROACH
% left_app ==> LEFT APPROACH

% INITIAL POSITION
% ==> Position from which the mounted camera scans the boundary box
%     to detect the charging port
%     THETA3 = pi/2 (Pointing straight ahead)
%     THETA4 = 0 (No tilt)

%% QUANTITY DEFINITIONS, UNITS & DIMENSIONS 

% all lengths & coords in meters wrt to charging station base
% all angles in radians
% all time in seconds

% TH1 ==> THETA1 measured from +ve X Axis
% TH2 ==> THETA2 measured from extension of LINK1 (CW)
% TH3 ==> THETA3 measured from +ve X Axis
% TH4 ==> THETA4 measured from +ve Y Axis

% Z ==> Z Coord of the end of LINK2 measured with respect to the vertical
%       center of the sliding mechanism

%% INITIAL VALUES REQUIRING CALCULATION

% SOLVING FOR REQUIRED JOINT ANGLES AT THE CHOSEN INITIAL POSITION 
% IN ACCORDANCE WITH CAMERA CONSTRAINT

 % At the chosen initial position, we require the camera to be at certain coords 
 % such that the complete bounding box is guaranteed to be in the FOV of
 % the camera.

% REQUIRED CAMERA COORDS
  cam_x = 0;
  cam_y = 0.15;
  cam_z = 0;

% CALCULATING REQUIRED OFFSET FOR LINK2

 % Since all inverse kinematics calculations are being done for the location
 % of LINK2's end, LINK2 must be offset such that IK calculations result in
 % a configuration that places the end-effector (camera in this case) 
 % at the required position.

 % Using the offset calculating function defined at the end of the file
  [offxi,offyi,offzi] = off(0.062,0,0,pi/2,0);
  L2xi = cam_x + offxi;
  L2yi = cam_y + offyi;
  L2zi = cam_z + offzi;

% USING INVERSE KINEMATICS TO OBTAIN ALL JOINT ANGLE SOLUTIONS 
% FOR THE CHOSEN INITIAL POSITION  (pair of (THETA1,THETA2) solutions)
  tmp_th1_mat = TH1_MLF{1}(L1,L2,L2xi,L2yi);  tmp_th1_mat_2 = TH1_MLF{2}(L1,L2,L2xi,L2yi);
  tmp_th2_mat = TH2_MLF{1}(L1,L2,L2xi,L2yi);  tmp_th2_mat_2 = TH2_MLF{2}(L1,L2,L2xi,L2yi);

%% NAMING VARIABLES FOR RIGHT & LEFT APPROACH JOINT ANGLES

if tmp_th2_mat<0 % then it is left approach
    TH1_i_c = tmp_th1_mat_2;  TH2_i_c = tmp_th2_mat_2;
    TH1_i_l_c = tmp_th1_mat;   TH2_i_l_c = tmp_th2_mat;
else
    TH1_i_c = tmp_th1_mat;     TH2_i_c = tmp_th2_mat;
    TH1_i_l_c = tmp_th1_mat_2;  TH2_i_l_c = tmp_th2_mat_2;    
end

%% INITIAL VALUES

% LINK LENGTHS
  L1 = 0.54; L2 = 0.475;

% JOINT ANGLE CONSTANTS FOR CHOSEN INITIAL POSITION
  % FOR left_app THETA1 will become PI-THETA1 & THETA2 will become -VE  
  TH3_i = pi/2; TH4_i = 0;

% Z AXIS RELEVANT VALUES AND CONSTANTS
  Z_i = L2zi; % value for chosen initial position such that camera is at Z=0
  z_chapp = 1.2; % value that z must take during chapp

% GIVEN PORT COORDINATES
  xpf_i = 0.35; ypf_i = 0.75; zpf_i = -0.1;

% GIVEN PORT ROTATIONS ABOUT XYZ AXES (+apx==UP) & (+cpz==CCW)
% assumption: no rotation of port about Y Axis
  apx = deg2rad(7.5); bpy = 0; cpz = deg2rad(5);

% GIVEN RANDOM INITIAL POSITION FOR THE END-EFFECTOR
  rx = 0.3; ry = 0.6; rz = 0.1;
% GIVEN RANDOM INITIAL VALUES FOR JOINT ANGLES THETA3 & THETA4
  TH3_ri = 0; TH4_ri = 0;

% GIVEN/TAKEN VELOCITY CONSTRAINTS
  max_tvel = 0.08; % translational
  max_avel = deg2rad(4); % rotational
  max_pvel = 0.01; % during plugging for the end-effector

% OTHER CONSTANTS & TIME CALCULATIONS
  ch_cyc = 32; % time for charging cycle
  
%% FINAL VALUES TO BE REACHED

% FINAL LINK2 COORDS SUCH THAT END-EFFECTOR REACHES IN FRONT OF PORT

  % All Inverse Kinematics Calculations have been done for the end of LINK2
  % such that THETA1 & THETA2 are calculated for a given position of LINK2
  
  % Offsets for the final position of LINK2 have to be calculated such that
  %  the end-effector reaches in front of the port.
  
  % The required offset vector given by off_v = [offxf,offyf,offzf] 
  %  is calculated as follows:
  
  % off_v is the -ve of the sum of two vectors:
  % 1) vk ==> vector from the end of LINK2 to the point where it cuts 
  %    the end-effector plane's normal vector at its tail
  %    (only k component is non-zero)

  % 2) vn ==> the normal vector of the end-effector plane

  % mag(vk) = dist. from LINK2 to JOINT4 + dist. from JOINT4 to tail of vn
  % mag(vn) = dist. from tail of vn to intermediate point on vn + 
  %           dist. from intermediate point to head of vn

  % Perpendicular Dist. between JOINT4 and END-EFFECTOR-NORMAL
    p_dist = 0.2285-0.145;

  % 1) dist. from LINK2 to JOINT4 = 0.145m
  %    (calculated from CAD model)
  % 2) dist. from JOINT4 to tail of vn ==> h = p_dist/cos(THETA4)
  % 3) dist. from tail of vn to intermediate point on vn 
  %    ==> b = p_dist/cot(THETA4) 
  % 4) dist. from intermediate point to head of vn ==> yos = 0.128m
  %     (calculated from CAD Model)
  
  % Assuming vn makes an angle THETA4 with the XY Plane and angle THETA3
  % with the +ve X Axis:
  % For proper alignment before plugging,
  %  THETA4 = -apx & 
  %  THETA3 = pi/2 + cpz

  %  Unit vector components of vn are as follows:
  %   i component ==> cos(THETA4)*cos(THETA3)
  %   j component ==> cos(THETA4)*sin(THETA3)
  %   k component ==> sin(THETA4)
  
  % FROM THE ABOVE CALCULATIONS
  h = p_dist/cos(-apx);
  b = p_dist/cot(-apx);
  yos = 0.128; % y for just off of surface of port
  dfp = 0.05; % distance_from_port to stop at just before plugging

  [offxf,offyf,offzf] = off(yos,p_dist,dfp,((pi/2)+cpz),-apx);
  L2xf = xpf_i + offxf;
  L2yf = ypf_i + offyf;
  L2zf = zpf_i + offzf;

  [offrx,offry,offrz] = off(yos,p_dist,0,TH3_ri,TH4_ri);
  L2rx = rx + offrx;
  L2ry = ry + offry;
  L2rz = rz + offrz;


% FINAL END-EFFECTOR JOINT ANGLES (THETA3 & THETA4) TO BE REACHED
  TH3_f = pi/2 + cpz; % THETA3 must be pi/2 + port rotation about Z Axis
  TH4_f = -apx; % THETA4 must be -ve of port rotation about X Axis

%% PLUGGING VALUES/CONSTANTS

% PLUGGING DISTANCE & COMPONENTS
  plug_dist = 0.033+dfp;
  xp_dist = plug_dist*(cos(-apx)*cos(pi/2 + cpz));
  yp_dist = plug_dist*(cos(-apx)*sin(pi/2 + cpz));
  zp_dist = plug_dist*sin(-apx);

% PLUGGING VELOCITIES & COMPONENTS
  plug_vel = 0.01;
  xp_vel = plug_vel*(cos(-apx)*cos(pi/2 + cpz));
  yp_vel = plug_vel*(cos(-apx)*sin(pi/2 + cpz));
  zp_vel = plug_vel*sin(-apx);

% PLUGGING TIME CALCULATION
  time_plug = plug_dist/plug_vel + 2; % 2sec buffer

%% IK for Random Initial Position
tmp_th1_mat = TH1_MLF{1}(L1,L2,L2rx,L2ry);  tmp_th1_mat_2 = TH1_MLF{2}(L1,L2,L2rx,L2ry);
tmp_th2_mat = TH2_MLF{1}(L1,L2,L2rx,L2ry);  tmp_th2_mat_2 = TH2_MLF{2}(L1,L2,L2rx,L2ry);

%% CHOOSE CORRECT THETA FOR RIGHT SIDE APPROACH
if tmp_th2_mat<0
    TH1_f = tmp_th1_mat_2;  TH2_f = tmp_th2_mat_2;
    TH1_fl = tmp_th1_mat;   TH2_fl = tmp_th2_mat;
else
    TH1_f = tmp_th1_mat;     TH2_f = tmp_th2_mat;
    TH1_fl = tmp_th1_mat_2;  TH2_fl = tmp_th2_mat_2;    
end

%% CHECK R/L LOCATION OF rx TO REACH R OR L APPROACH INITIAL POSITIONS

% CHOOSE LEFT APPROACH
if rx < 0
    l_app = 1;

    % Assign values to required angles
      TH1_ri = TH1_fl;  TH2_ri = TH2_fl;
      TH1_i_r = TH1_i_l_c;  TH2_i_r = TH2_i_l_c; % initial pos to go to from rand pos

% CHOOSE RIGHT APPROACH
else
    l_app = -1;

    % Assign values to required angles
      TH1_ri = TH1_f;   TH2_ri = TH2_f;
      TH1_i_r = TH1_i_c;  TH2_i_r = TH2_i_c; % initial pos to go to from rand pos
    
end

%% IK FOR FINAL POSITION OF END-EFFECTOR
tmp_th1_mat = TH1_MLF{1}(L1,L2,L2xf,L2yf); tmp_th1_mat_2 = TH1_MLF{2}(L1,L2,L2xf,L2yf);
tmp_th2_mat = TH2_MLF{1}(L1,L2,L2xf,L2yf); tmp_th2_mat_2 = TH2_MLF{2}(L1,L2,L2xf,L2yf);

%% CHOOSE THE CORRECT FINAL THETAS FOR THE PREVIOUSLY SELECTED APPROACH
if l_app == 1
    % CHECK FOR EDGE CASE
    if ((L2xf-0.5)^2 + L2yf^2 - (0.54)^2 < 0) && (L2xf < 0.5) && (L2yf > 0.5) %0.5 instead of 100
        ch_app = 1;

        % Need to interpolate from TH_ec_i to TH_i
        TH1_ec_i = TH1_i_r;  TH2_ec_i = TH2_i_r;
        TH1_i = TH1_i_c;  TH2_i = TH2_i_c;

        % CALCULATE TIME FOR INTERPOLATION
        time_ch_app = (abs(TH2_i_c - TH2_i_l_c)/(max_avel)) + 2; % 2sec buffer
        time_zchapp_up = abs(Z_i-z_chapp)/max_tvel; 
        time_zchapp_down = abs(L2zf-z_chapp)/max_tvel;
        time_ch_app = time_zchapp_up + time_ch_app + time_zchapp_down;
        
        % FINAL THETAS
        if tmp_th2_mat>0
            TH1_f = tmp_th1_mat;
            TH2_f = tmp_th2_mat;
        else
            TH1_f = tmp_th1_mat_2;
            TH2_f = tmp_th2_mat_2;
        end

        % CHECK FOR ANGLE WITH LARGEST ANGULAR DISPLACEMENT
          larg_adisp_ri_l = max(max(max(abs(TH1_ri-TH1_i_c),abs(TH2_ri-TH2_i_c)) ...
           ,abs((TH3_ri-(TH1_ri+TH2_ri))-(TH3_i-(TH1_i+TH2_i)))),abs(TH4_ri-TH4_i));
    
        % CALCULATE INTERPOLATION TIMES
         time_ri_ang = (larg_adisp_ri_l)/(max_avel) + 2; % 2sec buffer
         time_zri = abs(rz-Z_i)/max_tvel + 5; % 5sec buffer, wait at initial pos
         time_ri = time_ri_ang + time_zri;

        % CHECK FOR ANGLE WITH LARGEST ANGULAR DISPLACEMENT
        larg_adisp_if = max(max(max(abs(TH1_f-TH1_i),abs(TH2_f-TH2_i)) ...
            ,abs((TH3_f-(TH1_f+TH2_f))-(TH3_i-(TH1_i+TH2_i)))),abs(TH4_f-TH4_i));

        % CALCULATE TIME FOR INTERPOLATION
        time_if = (larg_adisp_if)/(max_avel);
        time_zif = abs(Z_i-L2zf)/max_tvel;
        time_if = max(time_if,time_zif) + 2; %2sec buffer


    else
        ch_app = -1;
        time_zchapp_up = 0;  time_zchapp_down = 0;
        time_ch_app = 0;
        TH1_ec_i = TH1_i_r;  TH2_ec_i = TH2_i_r;
        TH1_i = TH1_i_r; TH2_i = TH2_i_r;

        % FINAL THETAS
        if tmp_th2_mat<0
            TH1_f = tmp_th1_mat;
            TH2_f = tmp_th2_mat;
        else
            TH1_f = tmp_th1_mat_2;
            TH2_f = tmp_th2_mat_2;
        end

        % CHECK FOR ANGLE WITH LARGEST ANGULAR DISPLACEMENT
          larg_adisp_ri_l = max(max(max(abs(TH1_ri-TH1_i_c),abs(TH2_ri-TH2_i_c)) ...
           ,abs((TH3_ri-(TH1_ri+TH2_ri))-(TH3_i-(TH1_i+TH2_i)))),abs(TH4_ri-TH4_i));
    
        % CALCULATE INTERPOLATION TIMES
          time_ri_ang = (larg_adisp_ri_l)/(max_avel) + 2; % 2sec buffer
          time_zri = abs(rz-Z_i)/max_tvel + 5; % 5sec buffer, wait at initial pos
          time_ri = time_ri_ang + time_zri;

        % CHECK FOR ANGLE WITH LARGEST ANGULAR DISPLACEMENT
        larg_adisp_if = max(max(max(abs(TH1_f-TH1_i),abs(TH2_f-TH2_i)) ...
            ,abs((TH3_f-(TH1_f+TH2_f))-(TH3_i-(TH1_i+TH2_i)))),abs(TH4_f-TH4_i));
       
        % CALCULATE TIME FOR INTERPOLATION
        time_if = (larg_adisp_if)/(max_avel);
        time_zif = abs(Z_i-L2zf)/max_tvel;
        time_if = max(time_if,time_zif) + 2; % 2sec buffer

    end

elseif l_app == -1
    % CHECK FOR EDGE CASE
    if ((L2xf+0.5)^2 + L2yf^2 - (0.54)^2 < 0) && (L2xf > -0.5) && (L2yf > 0.5) %0.5 instead of 100
        ch_app = 1;

        % Need to interpolate from TH_ec_i to TH_i
        TH1_ec_i = TH1_i_r;  TH2_ec_i = TH2_i_r;
        TH1_i = TH1_i_l_c;  TH2_i = TH2_i_l_c;

        % CALCULATE TIME FOR INTERPOLATION
        time_ch_app = (abs(TH2_i_c - TH2_i_l_c)/(max_avel)) + 2; % 2sec buffer
        time_zchapp_up = abs(Z_i-z_chapp)/max_tvel;
        time_zchapp_down = abs(L2zf-z_chapp)/max_tvel;
        time_ch_app = time_zchapp_up + time_ch_app + time_zchapp_down;  

        % FINAL THETAS
        if tmp_th2_mat<0
            TH1_f = tmp_th1_mat;
            TH2_f = tmp_th2_mat;
        else
            TH1_f = tmp_th1_mat_2;
            TH2_f = tmp_th2_mat_2;
        end

        % CHECK FOR ANGLE WITH LARGEST ANGULAR DISPLACEMENT
          larg_adisp_ri = max(max(max(abs(TH1_ri-TH1_i_c),abs(TH2_ri-TH2_i_c)) ...
           ,abs((TH3_ri-(TH1_ri+TH2_ri))-(TH3_i-(TH1_i+TH2_i)))),abs(TH4_ri-TH4_i));

        % CALCULATE INTERPOLATION TIMES  
          time_ri_ang = (larg_adisp_ri)/(max_avel) + 2; % 2sec buffer
          time_zri = abs(rz-Z_i)/max_tvel + 5; % 5sec buffer, wait at initial pos
          time_ri = time_ri_ang + time_zri;

        % CHECK FOR ANGLE WITH LARGEST ANGULAR DISPLACEMENT
        larg_adisp_if = max(max(max(abs(TH1_f-TH1_i),abs(TH2_f-TH2_i)) ...
            ,abs((TH3_f-(TH1_f+TH2_f))-(TH3_i-(TH1_i+TH2_i)))),abs(TH4_f-TH4_i));

        % CALCULATE TIME FOR INTERPOLATION
        time_if = (larg_adisp_if)/(max_avel);
        time_zif = abs(Z_i-L2zf)/max_tvel;
        time_if = max(time_if,time_zif) + 2; % 2sec buffer    

    else
        ch_app = -1;
        time_zchapp_up = 0;  time_zchapp_down = 0;
        time_ch_app = 0;
        TH1_ec_i = TH1_i_r;  TH2_ec_i = TH2_i_r;
        TH1_i = TH1_i_r; TH2_i = TH2_i_r;

        % FINAL THETAS
        if tmp_th2_mat>0
            TH1_f = tmp_th1_mat;
            TH2_f = tmp_th2_mat;
        else
            TH1_f = tmp_th1_mat_2;
            TH2_f = tmp_th2_mat_2;
        end
        
        % CHECK FOR ANGLE WITH LARGEST ANGULAR DISPLACEMENT
          larg_adisp_ri = max(max(max(abs(TH1_ri-TH1_i_c),abs(TH2_ri-TH2_i_c)) ...
           ,abs((TH3_ri-(TH1_ri+TH2_ri))-(TH3_i-(TH1_i+TH2_i)))),abs(TH4_ri-TH4_i));

        % CALCULATE INTERPOLATION TIMES  
          time_ri_ang = (larg_adisp_ri)/(max_avel) + 2; % 2sec buffer
          time_zri = abs(rz-Z_i)/max_tvel + 5; % 5sec buffer, wait at initial pos
          time_ri = time_ri_ang + time_zri;

        % CHECK FOR ANGLE WITH LARGEST ANGULAR DISPLACEMENT
        larg_adisp_if = max(max(max(abs(TH1_f-TH1_i),abs(TH2_f-TH2_i)) ...
            ,abs((TH3_f-(TH1_f+TH2_f))-(TH3_i-(TH1_i+TH2_i)))),abs(TH4_f-TH4_i));

        % CALCULATE TIME FOR INTERPOLATION
        time_if = (larg_adisp_if)/(max_avel);
        time_zif = abs(Z_i-L2zf)/max_tvel;
        time_if = max(time_if,time_zif) + 2; % 2sec buffer

    end
end

function [offx,offy,offz] = off(yos,p_dist,dfp,TH3,TH4)
h = p_dist/cos(TH4);
b = p_dist/cot(TH4);

vn_sclr = yos+dfp+b; % scaling factor for unit vector vn
offx = -(vn_sclr)*(cos(TH4)*cos(TH3));
offy = -(vn_sclr)*(cos(TH4)*sin(TH3));
offz = -(vn_sclr)*(sin(TH4)) + 0.145 + h;
end

