%This program calculates the distribution of the laser beam in horizontal
%and vertical field of view.
%Done by Vignesh Balasubramani
%Technische Hochschule Ingolstadt
%04 Dec 2021
%addional functions needed are
% distance.m
% distribution.m
% side_distribution.m
clear all; close all; clc;
%-------INPUTS-----------------
lane_width=3.25;%width of road
v0=[100:5:130]*5/18;%velocity range of VUT0 in kmph
v1=[60:5:90]*5/18;%velocity range of TUV1 in kmph
v2=[70:5:120]*5/18;%velcity range of TUV2 in kmph
v3=[105:5:160]*5/18;%velocity range of TUV3 in kmph
v4=[105:5:135]*5/18;%velocity range of TUV4 in kmph
dimVUT=[5125,1900,1496]/1000;%dimesion of VUT [length width height] in m
dimTSV=[11400,2400,3540;4060,1800,1430;4060,1800,1430;4060,1800,1430]/1000;%dimension of TUV [length width height] in mm
TTC12=1:5;%TTC between TUV1 and TUV2 in s
TTC02=0.7:0.1:5;%TTC between VUT0 and TUV2 in s
TTC03=0.5:0.1:1.0;%TTC between VUT0 and TUV3 in s
TTC04=0.7:0.1:5;%TTC between VUT0 and TUV4 in s
%-------SCENARIO SETUP--------------------
%first column values: +1 for forward and -1 for backward w.r.t VUT0
%second column defines the lane of the TUV (1,2 or 3) starting from the bottom
scenario=[1 1;1 1;-1 3;-1 2];
loc=[0 0];%the location of VUT0 is at the centre of lane 2. 
% loc=[0 -(1.5*lane_width-(dimVUT(2)/2))]; % toggle for VUT0 at extreme of lane 1.
% loc=[0 (1.5*lane_width-(dimVUT(2)/2))]; % toggle for VUT0 at extreme of lane 3.
%-------DISTANCE CALCULATION---------------
dist=zeros(4,2);%contains the minimum and maximum possible distance between VUT0 and TUV
[c,d]=distance(v0,v2,TTC02); %calls the function 'distance.m'
[a,b]=distance(v1,v2,TTC12);
[e,f]=distance(v0,v3,TTC03);
[g,h]=distance(v0,v4,TTC04);
dist(1,:)=[a,b]+[c,d]+[1 1]*(dimTSV(2,1)+dimVUT(1)/2+dimTSV(1,1)/2);
dist(2,:)=[c,d]+[1,1]*(dimVUT(1)/2+dimTSV(2,1)/2);
dist(3,:)=[e,f]+[1,1]*(dimVUT(1)/2+dimTSV(3,1)/2); 
% dist(3,:)=[<min_distance>,<max_distance>]; %min and max possible dist of TUV3 can also be specified directly
dist(4,:)=[g,h]+[1,1]*(dimVUT(1)/2+dimTSV(4,1)/2);
%------DISTRIBUTION CALCULATION--------------------------
D2=distribution_highway(dist(2,:),lane_width,dimVUT,dimTSV(2,:),loc,scenario(2,:));
D3=side_distribution(dist(3,:),lane_width,dimVUT,dimTSV(3,:),loc,scenario(3,:));
D4=distribution_highway(dist(4,:),lane_width,dimVUT,dimTSV(4,:),loc,scenario(4,:));
%-----------Writing to EXCEL---------------------
writematrix(D2,'Distribution_highway.xlsx','Sheet',1,'Range','B3:U3');
writematrix(D3,'Distribution_highway.xlsx','Sheet',1,'Range','B4:U4');
writematrix(D4,'Distribution_highway.xlsx','Sheet',1,'Range','B5:U5');
%------------END---------------------------








