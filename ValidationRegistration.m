%Validation Data Processing Script Strategy 2
%Registration
%26 6 2020
clear all
close all
clc

%% Register NDI/RCM Frames from NDI stationary dataset

Reg = load('NDI_dataRegistration26Jun2020.mat');
Reg = Reg.NDI_data;
%RCM from World Frame
RCM_frame = [0 0 -1 -300.71; 0 1 0 61; 1 0 0 -7; 0 0 0 1];

%World position and z axis of ABCD frame trackers:
tW_A = [20 -100 15]';
tW_B = [-20 -100 0]'; 
tW_C = [0 -100 15]';
tW_D = RCM_frame*[-11.85 0 0 1]';
tW_D = tW_D(1:3);

zW_A = [1 0 0]'; 
zW_B = [-1 0 0]'; 
zW_C = [0 0 1]'; 
zW_D = RCM_frame(1:3,1:3)*[1 0 0]';

% These points correspond to those ones.
zNDI_abcd = [reshape(Reg.A.T(1:3,3,:),[3,Reg.N,1]), ...
    reshape(Reg.B.T(1:3,3,:),[3,Reg.N,1]), ...
    reshape(Reg.C.T(1:3,3,:),[3,Reg.N,1]), ...
    reshape(Reg.D.T(1:3,3,:),[3,Reg.N,1])];
zW_abcd = [repmat(zW_A,1,Reg.N) repmat(zW_B,1,Reg.N) ...
           repmat(zW_C,1,Reg.N) repmat(zW_D,1,Reg.N)];

tNDI_abcd = [reshape(Reg.A.T(1:3,4,:),[3,Reg.N,1]), ...
    reshape(Reg.B.T(1:3,4,:),[3,Reg.N,1]), ...
    reshape(Reg.C.T(1:3,4,:),[3,Reg.N,1]), ...
    reshape(Reg.D.T(1:3,4,:),[3,Reg.N,1])];
tW_abcd = [repmat(tW_A,1,Reg.N) repmat(tW_B,1,Reg.N) ...
           repmat(tW_C,1,Reg.N) repmat(tW_D,1,Reg.N)];
       
%Create Data matrices combining all that information
pNDI_av = mean(tNDI_abcd,2);
pW_av = mean(tW_abcd,2);

%Now define Data Matrices based on points and z axis info Stack
Mb = [(tNDI_abcd - pNDI_av) zNDI_abcd];
Ma = [(tW_abcd - pW_av) zW_abcd];

%Solve rotation with a pseudo inverse/least squares approach
%Ra*Mb = Ma, rank(Mb) == 3
[U,L,V] = svd(Ma*Mb'/(Mb*Mb'));
% disp(norm(eye(3)-L))

%solve rotation
W_R_ndi = U*V;
%W_R_ndi = Rx(pi/2)*Rz(-pi/2)*Rx(deg2rad(15))*Ry(deg2rad(-5))*Rx(deg2rad(-0.25)); %Really Good Guess

%Now solve translation
W_t_ndi = tW_abcd - W_R_ndi * tNDI_abcd;
W_t_ndi = mean(W_t_ndi,2);

%Now use my data fusion function to find the transform:
TW_NDI = [W_R_ndi W_t_ndi; 0 0 0 1];

%Use Matlab to fix errors in the previous estimate:
ptCloudNDI = pointCloud(tNDI_abcd');
ptCloudRaven = pointCloud(tW_abcd');
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
text(TW_NDI(1,4)+10,TW_NDI(2,4),TW_NDI(3,4),'NDI')
%Plot The RCM Frame:
plotcoord3(RCM_frame,50,'r','g','b')
text(RCM_frame(1,4)+10,RCM_frame(2,4),RCM_frame(3,4)+25,'RCM')

%Plot NDI Trajectories
Pa = TransformPoints(TW_NDI,[Reg.A.x,Reg.A.y,Reg.A.z]);
Pb = TransformPoints(TW_NDI,[Reg.B.x,Reg.B.y,Reg.B.z]);
Pc = TransformPoints(TW_NDI,[Reg.C.x,Reg.C.y,Reg.C.z]);
Pd = TransformPoints(TW_NDI,[Reg.D.x,Reg.D.y,Reg.D.z]);

plot3(Pa(:,1),Pa(:,2),Pa(:,3),'b.'), hold on
plot3(Pb(:,1),Pb(:,2),Pb(:,3),'r.'), hold on
plot3(Pc(:,1),Pc(:,2),Pc(:,3),'g.'), hold on
plot3(Pd(:,1),Pd(:,2),Pd(:,3),'m.'), hold on

%Plot The Base Markers:
Pabcd = [tW_A tW_B tW_C tW_D]';
plot3(Pabcd(:,1),Pabcd(:,2),Pabcd(:,3),'k.'), hold on
text(tW_A(1)+1,tW_A(2)+1,tW_A(3)+1,'A')
text(tW_B(1)+1,tW_B(2)+1,tW_B(3)+1,'B')
text(tW_C(1)+1,tW_C(2)+1,tW_C(3)+1,'C')
text(tW_D(1)+1,tW_D(2)+1,tW_D(3)+1,'D')

%Figure style
title('3D View (X-Y-Z) of NDI tracker Registration'), xlabel('X displacement')
ylabel('Y displacement'), zlabel('Z displacement')
view(3)
axis equal
grid on

%%


%Short cut through registration
% TW_NDI = [0.0352    0.9656   -0.2576 -201.6408;
%          -0.0023   -0.2577   -0.9662 -370.0656;
%          -0.9994    0.0346   -0.0068  -16.3418;
%             0         0         0    1.0000];
        
        
        