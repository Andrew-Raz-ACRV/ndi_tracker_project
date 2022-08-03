%Validation Data Processing Script Strategy 2
%26 6 2020
clear all
close all
clc

%% Register NDI/RCM Frames from NDI stationary dataset

% Short cut through registration from other Script:
TW_NDI = [0.0352    0.9656   -0.2576 -201.6408;
         -0.0023   -0.2577   -0.9662 -370.0656;
         -0.9994    0.0346   -0.0068  -16.3418;
            0         0         0    1.0000];
        
%% Load NDI and SnakeRaven ROS data:  
        