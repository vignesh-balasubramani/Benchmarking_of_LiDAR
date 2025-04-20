%This program calculates the distribution of the laser beam in horizontal
%and vertical field of view.
%for rural scenario
%Done by Vignesh Balasubramani
%Technische Hochschule Ingolstadt
%26 Dec 2021
%addional functions needed are
% distance.m
% distribution_rural.m
% side_distribution_rural.m
clear all; close all; clc;
%-------INPUTS-----------------
lane_width=3.25;%width of road
v0=[70:5:100]*5/18;%velocity range of VUT0 in kmph
v1=[20:5:60]*5/18;%velocity range of TUV1 in kmph
v2=[-70:-5:-110]*5/18;%velcity range of TUV2 in kmph
v3=[80:5:110]*5/18;%velocity range of TUV3 in kmph
dimVUT=[5125,1900,1496]/1000;%dimesion of VUT [length width height] in m
dimTSV=[3835,1920,2490;4060,1800,1430;1420,750,1800]/1000;%dimension of TUV [length width height] in mm
TTC01=0.7:0.1:5;%TTC between VUT0 and TUV1 in s
TTC02=0.7:0.1:5;%TTC between VUT0 and TUV2 in s
TTC03=0.5:0.1:1.0;%TTC between VUT0 and TUV3 in s
%-------SCENARIO SETUP--------------------
%first column values: +1 for forward and -1 for backward w.r.t VUT0
%second column defines the lane of the TUV (1 or 2) starting from the bottom
scenario=[1 1;1 2;-1 2];
% loc=[0 0];%the location of VUT0 is at the centre of lane 1. 
loc=[0 0.5*lane_width-dimVUT(2)/2];
%-------DISTANCE CALCULATION---------------
dist=zeros(3,2);%contains the minimum and maximum possible distance between VUT0 and TUV
[a,b]=distance(v0,v1,TTC01); %calls the function 'distance.m'
[c,d]=distance(v0,v2,TTC02);
[e,f]=distance(v0,v3,TTC03);
dist(1,:)=[a,b]+[1 1]*(dimVUT(1)/2+dimTSV(1,1)/2);
dist(2,:)=[c,d]+[1,1]*(dimVUT(1)/2+dimTSV(2,1)/2);
dist(3,:)=[e,f]+[1,1]*(dimVUT(1)/2+dimTSV(3,1)/2); 
%------DISTRIBUTION CALCULATION--------------------------
D1=distribution_rural(dist(1,:),lane_width,dimVUT,dimTSV(1,:),loc,scenario(1,:));
D2=distribution_rural(dist(2,:),lane_width,dimVUT,dimTSV(2,:),loc,scenario(2,:));
D3=side_distribution_rural(dist(3,:),lane_width,dimVUT,dimTSV(3,:),loc,scenario(3,:));
%-----------Writing to EXCEL---------------------
writematrix(D1(1:8),'Distribution_rural.xlsx','Sheet',1,'Range','B3:I3');
writematrix(D1(9:14),'Distribution_rural.xlsx','Sheet',1,'Range','P3:U3');
writematrix(D2(1:8),'Distribution_rural.xlsx','Sheet',1,'Range','B4:I4');
writematrix(D2(9:14),'Distribution_rural.xlsx','Sheet',1,'Range','P4:U4');
writematrix(D3,'Distribution_rural.xlsx','Sheet',1,'Range','B5:U5');
%------------END---------------------------








