function PD = Fun_PD_KS_LaunchLand
%-----------------------------------------------------------------------------
% Edited 18/10/2022 by Benjamin Tummon, Undergraduate Engineer QUT           %
% Project   : Kite Control for Green Energy                                  %
% Changes   : Added parameters for setting launching and landing control     %
%-----------------------------------------------------------------------------
%-----------------------------------------------------------------------------
% Project   : LAKSA                                                          %
% Authors   : Gonzalo Sanchez-Arriaga, Alejandro Pastor-Rodriguez,           %
% Language  : Matlab                                                         %
% Synopsis  : Physical Parameters                                            %
% Copyright:  Universidad Carlos III de Madrid, 2017. All rights reserved    %
%-----------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Inputs: No inputs                                                %%
%                                                                    %%
%   Outputs: structure PD with the physical parameters               %% 
%                                                                    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kite Geometry and Inertia  
PD.Kite.c     =  1.4;            % Kite Chord                          (m)
PD.Kite.b     =  4.3;            % Kite span                           (m)
PD.Kite.m     =  3.4;            % Kite mass                           (kg)
PD.Kite.h     =  2.2;            % Kite height                         (m, only plotting purposes and experimental comparison)
PD.Kite.hg    =  1.38;           % Z-Distance between G and kite tips  (m, only plotting purposes)
PD.Kite.A     =  10;             % Kite surface                        (m^2)  
PD.Kite.Ix    =  8.68;           % Ix                                  (kg m^2)
PD.Kite.Iy    =  2.43;           % Iy                                  (kg m^2)
PD.Kite.Iz    =  8.40;           % Iz                                  (kg m^2) 
PD.Kite.Ixz   =  0.33;           % Ixz                                 (kg m^2)

% Force Aerodynamic coefficients
PD.Aero.Full   =  0;             % Set 1 to use full model   

PD.Aero.Cx0    = -0.065;             % Cx0                          (-)  
PD.Aero.Cxalfa =  0.18;              % Cx_alfa                      (-)  
PD.Aero.Cybeta = -1.57;              % Cy_beta                      (-) 
PD.Aero.Cz0    =  0.12;              % Cz0                          (-)  
PD.Aero.Czalfa = -2.97;              % Cz_alfa                      (-)  

% Torque Aerodynamic coefficients
PD.Aero.Clbeta =  1.24;                            % Cl_beta                      (-)
PD.Aero.Clp    = -0.15;                            % Cl_p_tilde                   (-)      
PD.Aero.Cm0    =  0.13;                            % Cm0                          (-)      
PD.Aero.Cmalfa = -0.76;                            % Cm_alfa                      (-)      
PD.Aero.Cmq    = -0.17;                            % Cm_q_tilde                   (-)       
PD.Aero.Cnbeta =  0.78;                            % Cn_beta                      (-)      
PD.Aero.Cnr    = -0.002;                           % Cn_r_tilde                   (-)         

PD.Aero.Vref      =  7;             % V_ref                        (m/s)    

% Control ->  This control is not implemented in the current version of the code (except Cn_delta_r)
PD.Aero.Cydelta_r = 0;
PD.Aero.Cmdelta_e = 0;
PD.Aero.Cldelta_a = 0;
PD.Aero.Cldelta_r = 0;
PD.Aero.Cndelta_r = 0.04; % The code assumes that a tension difference acts as a rudder deflection 

% Aerodynamic Model Limits (only for postprocess checking purposes)
PD.Aero.alfa_s =  25;            % Stall angle                  (�)
PD.Aero.beta_m =  15;            % Maximum sideslip angle       (�)

% Environment parameters
PD.Env.g      =  9.8;            % Earth acceleration                (m/s^2)
PD.Env.ro     =  1.15;           % Air density                       (kg/m^3)
PD.Env.Type   =  0;              % 0 -> Wind Velocity is constant
                                 % 1 -> Wind Speed = Vw*(h/H0)^alfa*(1+eps*sin(Omega*t))
                        
                                 
PD.Env.Vw     = 7;               % Wind Velocity                     (m/s)
PD.Env.alfa   = 0.14;            % Exponent of the wind speed law
PD.Env.H0     = 10;              % Height Scale                      (m)
PD.Env.eps    = 0.1;             % Wind Speed fluctuation level
PD.Env.Omega  = 0.6;             % Wind Speed fluctuation frequency  (rad/s)

% Tether characteristics
PD.Tether.Ll  = 23.85;           % Length of the leading edge tether (line+bridle)   (m)    
PD.Tether.Lt  = 23.19;           % Length of the trailing edge tether (line+bridle)  (m)
PD.Tether.E   = 10e9;            % Tether Young's Modulus                            (Pa)
PD.Tether.dt  = 1.5e-3;          % Tether diameter                                   (m)     
PD.Tether.ft  = 0.1;            % Dimensionless friction coefficient 

PD.Tether.XA  = 0.42;            % X-Body coordinate of Attachment point A (m) 
PD.Tether.YA  = 1.05;            % Y-Body coordinate of Attachment point A (m) 
PD.Tether.ZA  =-0.20;            % Z-Body coordinate of Attachment point A (m) 

PD.Tether.XB  =-0.97;            % X-Body coordinate of Attachment point B (m) 
PD.Tether.YB  = 2.15;             % Y-Body coordinate of Attachment point B (m) 
PD.Tether.ZB  = 1.38;             % Z-Body coordinate of Attachment point B (m)

% Control Bar Geometry
PD.Bar.Lc     = 0.56;            % Length og the control Bar
PD.Bar.Ls     = 1.51;            % Distance between OE and O_{EF}  
PD.Bar.Lds    = 0.52;            % Distance between OE and the depower stopperball     
PD.Bar.Lps    = 0.54;            % Distance between O_EF and the power stopperball 

% Control parameters for trailing edge tethers
PD.Ctr.Type   = 0;               % 0 ->  No control
                                 % 1 ->  Pull Up   -> PR     = PR0+(PRf-PR0)*(1+tanh((t-Ts)/Tr))/2;    
                                 % 2 ->  Sinusoidal Steering  -> lambda = Lam*sin(OmLam*t) 
                                 % 3 ->  Close Loop 
                                 % 4 - > Launching from ground (added by Ben Tummon 18/10/2022)
                                 % 5 - > Landining to ground (added by Ben Tummon 18/10/2022)
                                 
PD.Ctr.Lam    = 2.5;             % lambda_0       (deg)
PD.Ctr.OmLam  = 0.32;            % omega_lambda   (rad/s)
PD.Ctr.PR0    = 0.5;             % PR0            Initial Power Ratio         
PD.Ctr.PRf    = 0.75;            % PRf            Final Power ratio
PD.Ctr.Tr     = 1.0;             % Tr             Characteristic Raising time of the bar (s) 
PD.Ctr.Ts     = 8.;              % Ts             Time shift  (s) 
PD.Ctr.TL     = 1.5;             % Characteristic Time in the steering manoeuvre 
PD.Ctr.TC     = 1.2;             % Characteristic Time in the steering manoeuvre 

PD.Ctr.GD     = -1.0;  
PD.Ctr.GP     = -1.0;
PD.Ctr.GI     = -1.0;

PD.Ctr.Y0     = 0.2*PD.Tether.Ll;                % Y0       for Close Loop
PD.Ctr.Om     = 0.5*sqrt(PD.Env.g/PD.Tether.Ll); % omega_Y  for Close Loop


% Numerical Parameters
PD.Num.RelTol  = 1e-8;           % Integrator Relative Tolerance
PD.Num.AbsTol  = 1e-10;          % Integrator Absolute Tolerance
PD.Num.NewTol  = 5e-8;           % Newton-Raphson Tolerance
PD.Num.MaxIt   = 100;            % Newton-Raphson Maximum number of iterations
PD.Num.dh      = 1e-6;           % Numerical Jacobian step




end