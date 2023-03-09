# ndi_tracker_project
Code in CombinedAPISampleDlg.cpp is modified NDI software to create a .csv file from four sensors storing translation and rotation measurements in mm and quaternions (Sample:,Hour,Minutes,Seconds,ms,Xa,Ya,Za,qoa,qxa,qya,qza,Xb,Yb,Zb,qob,qxb,qyb,qzb,Xc,Yc,Zc,qoc,qxc,qyc,qzc,Xd,Yd,Zd,qod,qxd,qyd,qzd)

Search 'Andrew' to find the specific modifications, see the .csv file as an example of data

There is Matlab code to process the data from .csv to vectors using the command NDI_csr_to_vectors.m
Other codes include examples of performing registration with known points for the sensors.

For students at QUT, there is an instruction guide pdf in this repository
