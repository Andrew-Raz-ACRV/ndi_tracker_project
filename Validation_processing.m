%Validation Data Processing Script
%Combines the rosbag and NDI data into one:
clear all
close all
clc

%% Load the NDI and ROS data

load('NDI_data25Jun2020calibratedmotions.mat');
load('SnakeRaven_data25Jun2020calibratedmotions.mat');
S = SnakeRaven_data;

%% Register the NDI data to the Raven WORLD frame
%{
We know that sensor A,B,C were static attached to the Raven at:

D: [0 0 -8] -z axis of tool frame
%}

%World position and z axis of base frame trackers:
tW_A = [20 -100 15]'; %[10 0 0]';
tW_B = [-20 -100 0]'; %[-10 0 0]';
tW_C = [0 -100 15]';%[0 0 10]';

zW_A = [1 0 0]'; %[1 0 0]'; 
zW_B = [-1 0 0]'; %[-1 0 0]'; 
zW_C = [0 0 1]'; %[1 0 0]'; 

%Solve the rotation matrix using the z-axis information and translation
%information
%zW_abc = W_R_ndi * zNDI_abc
%zW_abc(zNDI_abc)^-1 = W_R_ndi

%These points correspond to those ones.
zNDI_abc = [reshape(NDI_data.A.T(1:3,3,:),[3,NDI_data.N,1]), reshape(NDI_data.B.T(1:3,3,:),[3,NDI_data.N,1]), ...
    reshape(NDI_data.C.T(1:3,3,:),[3,NDI_data.N,1])];
zW_abc = [repmat(zW_A,1,NDI_data.N) repmat(zW_B,1,NDI_data.N) repmat(zW_C,1,NDI_data.N)];

tNDI_abc = [reshape(NDI_data.A.T(1:3,4,:),[3,NDI_data.N,1]), reshape(NDI_data.B.T(1:3,4,:),[3,NDI_data.N,1]), ...
    reshape(NDI_data.C.T(1:3,4,:),[3,NDI_data.N,1])];
tW_abc = [repmat(tW_A,1,NDI_data.N) repmat(tW_B,1,NDI_data.N) repmat(tW_C,1,NDI_data.N)];

%% My Registration Method:
%Create Data matrices combining all that information
pNDI_av = mean(tNDI_abc,2);
pW_av = mean(tW_abc,2);

%Now define Data Matrices based on points and z axis info Stack
% W_R_ndi * (tNDI_abc - pNDI_av) = (tW_abc - pW_av)
% W_R_ndi * zNDI_abc = zW_abc
% W_R_ndi * Mb = Ma
Mb = [(tNDI_abc - pNDI_av) zNDI_abc];
Ma = [(tW_abc - pW_av) zW_abc];

%Solve rotation with a pseudo inverse/least squares approach
% W_R_ndi * Mb = Ma
% W_R_ndi * Mb * Mb' = Ma * Mb'
% W_R_ndi = Ma * Mb' * (Mb * Mb')^-1
% W_R_ndi = Ma * Mb' / (Mb * Mb');

%Ra*Mb = Ma, rank(Mb) == 3
[U,L,V] = svd(Ma*Mb'/(Mb*Mb'));
%disp(norm(eye(3)-L))

%solve rotation
% W_R_ndi = U*V;

W_R_ndi = Rx(pi/2)*Rz(-pi/2)*Rx(deg2rad(15))*Ry(deg2rad(-5))*Rx(deg2rad(-0.25)); %Really Good Guess

%Now solve translation
% W_t_abc = W_R_ndi * ndi_t_abc + W_t_ndi
% W_t_ndi = W_t_abc - W_R_ndi * ndi_t_abc

W_t_ndi = tW_abc - W_R_ndi * tNDI_abc;
W_t_ndi = mean(W_t_ndi,2);

%Now use my data fusion function to find the transform:
TW_NDI = [W_R_ndi W_t_ndi; 0 0 0 1];

%% Approach two: MATLAB ICP PointCloud registration:

ptCloudNDI = pointCloud(tNDI_abc');
ptCloudRaven = pointCloud(tW_abc');
ptCloudNDIDownsampled = pcdownsample(ptCloudNDI,'gridAverage',0.1);

tform = pcregrigid(ptCloudNDIDownsampled,ptCloudRaven,'InitialTransform',affine3d(TW_NDI'));
TW_NDI = tform.T';

%% Now plot both datasets in one figure relative to the WORLD

figure(1)
%Plot World
plotcoord3(eye(4),100,'r','g','b')
text(0,0,0,'WORLD')
%Plot NDI Frame
plotcoord3(TW_NDI,50,'r','g','b')
text(TW_NDI(1,4),TW_NDI(2,4),TW_NDI(3,4),'NDI')
%Plot The RCM Frame:
RCM_frame = [0 0 -1 -300.71; 0 1 0 61; 1 0 0 -7; 0 0 0 1];
plotcoord3(RCM_frame,50,'r','g','b')
text(RCM_frame(1,4)+10,RCM_frame(2,4),RCM_frame(3,4)+25,'RCM')

%Plot NDI Trajectories
Pa = TransformPoints(TW_NDI,[NDI_data.A.x,NDI_data.A.y,NDI_data.A.z]);
Pb = TransformPoints(TW_NDI,[NDI_data.B.x,NDI_data.B.y,NDI_data.B.z]);
Pc = TransformPoints(TW_NDI,[NDI_data.C.x,NDI_data.C.y,NDI_data.C.z]);
Pd = TransformPoints(TW_NDI,[NDI_data.D.x,NDI_data.D.y,NDI_data.D.z]);

%Compute endeffector transforms
D_T_end = [Rx(pi) [0 0 -8]'; 0 0 0 1];
W_T_D = TransformTransforms(TW_NDI,NDI_data.D.T); %W_T_D = W_T_NDI * NDI_T_D;
W_T_end = TransformsTransform(W_T_D,D_T_end); %W_T_end = W_T_D * D_T_end;

Px = reshape(W_T_end(1,4,:),[length(W_T_end),1,1]);
Py = reshape(W_T_end(2,4,:),[length(W_T_end),1,1]);
Pz = reshape(W_T_end(3,4,:),[length(W_T_end),1,1]);

% plot3(Pa(:,1),Pa(:,2),Pa(:,3),'b.'), hold on
% plot3(Pb(:,1),Pb(:,2),Pb(:,3),'r.'), hold on
% plot3(Pc(:,1),Pc(:,2),Pc(:,3),'g.'), hold on
plot3(Px,Py,Pz,'m.'), hold on

%Plot The Base Markers:
% Pabc = [tW_A tW_B tW_C]';
% plot3(Pabc(:,1),Pabc(:,2),Pabc(:,3),'k.'), hold on
% text(tW_A(1)+1,tW_A(2)+1,tW_A(3)+1,'A')
% text(tW_B(1)+1,tW_B(2)+1,tW_B(3)+1,'B')
% text(tW_C(1)+1,tW_C(2)+1,tW_C(3)+1,'C')

%Plot The SnakeRaven Forward Kinematics Trajectories
n = length(S.FK);
x = reshape(S.FK(1,4,:),[n,1]);
y = reshape(S.FK(2,4,:),[n,1]);
z = reshape(S.FK(3,4,:),[n,1]);
plot3(x,y,z,'b.'), hold on

%TOO SLOW TOO MANY DATA
% for ii=1:length(S.FK)
%     %plot transform
%     plotcoord3(S.FK(:,:,ii),5,'r','g','b')
% end

%Plot Anatomy or voxels?

%Figure style
title('3D View (X-Y-Z) of NDI tracker and SnakeRaven Trajectories'), xlabel('X displacement')
ylabel('Y displacement'), zlabel('Z displacement')
view(3)
axis equal
grid on

%Plot Service Sphere
% GenerateServicesphere(W_T_end); %cat(3,NDI_data.A.T,NDI_data.D.T)
