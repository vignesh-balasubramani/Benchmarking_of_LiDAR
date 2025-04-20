%This program calculates the distribution of the laser beam in horizontal
%and vertical field of view.
%Done by Vignesh Balasubramani
%Technische Hochschule Ingolstadt
%11 Jan 2022
%addional functions needed are
% distribution_VRU.m
% distribution_urban.m
% af_cyc.m
% af_ad.m
clear all; close all; clc;
%------------------INPUTS----------------------
b1=3.25; %width of the lane the VUT is in
b2=3.25; %width of the perpendicular lane
zebra=2; %width of zebra crossing
gap=0.5; %gap of zebra crossing from the end of road
dimVUT=[5125,1900,1496]/1000; %length breadth height of VUT
dimVRU=[1910,500,1860;600,500,1800;711,298,1154]/1000; %cyclist, adult, child
dimTSV=[4060,1800,1430;4060,1800,1430;4060,1800,1430]/1000; %length width and height of TSV
v0=[0:5:50]*5/18;%velocity range of VUT0 in kmph
v1=[0:5:60]*5/18;%velocity range of TUV1 in kmph
v2=[0:5:60]*5/18;%velcity range of TUV2 in kmph
v3=[10:5:25]*5/18;%velocity range of TUV3 in kmph
TTC01=0.7:0.1:5;%TTC between VUT0 and TUV1 in s
TTC02=0.7:0.1:5;%TTC between VUT0 and TUV2 in s
TTC03=0.7:0.1:5;%TTC between VUT0 and TUV3 in s
loc=[-(b2+gap+zebra+dimVUT(1)/2),-(b1/2)]; %location of the laser
%------calculation of area factors----------
AF_cyc=af_cyc(1910,500,1860);
AF_adult=af_ad(600,500,1800,1);% (lenght,width,height,1 for adult)
AF_child=af_ad(711,298,1154,0);% (lenght,width,height,0 for child)
%--------DISTRIBUTION CALCULATION-------------
D_VRU=zeros(3,12);
for j=1:3
    D_VRU(j,:)=distribution_VRU(dimVUT,dimVRU(j,:),b1,b2,zebra,gap,loc);
end
max_dist=[max(TTC01)*max(v1),max(TTC02)*max(v2),max(TTC03)*max(v3)];
D_TSV=zeros(3,12);
for i=1:3
    D_TSV(i,:)=distribution_urban(dimVUT,dimTSV(i,:),b1,b2,zebra,gap,loc,max_dist(i),i);
end
%----------WRITING IN EXCEL--------------------
writematrix(D_VRU,'Distribution_urban.xlsx','Sheet',2,'Range','B3:M5');
writematrix(D_TSV,'Distribution_urban.xlsx','Sheet',1,'Range','B3:M5');
