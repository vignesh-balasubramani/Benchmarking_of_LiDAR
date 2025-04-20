function Distribution=distribution(x,b,dim0,dim1,loc,scenario)
%Distribution=distribution(min_max_x_distance,lane_width,dimension of VUT0,dimesion of TUV, location of laser,scenario definition)
%function calculates the max and min distribution of the laser necessary
%-------inputs-------
%     x: [min dist, max dist]
%     b: lane width
%     dim0: dimension of VUT0
%     dim1: dimension of TUV
%     loc: location of the laser
%     scenario: scenari defenition of the TUV
%-------outputs---------
%         max_angle: maximum sweep angle including symmetry (degrees)
%         min_angle: maximum sweep angle including symmetry (degrees)
%         max_density: maximum Horizontal density required (rays/degree)
%         min_density: minimum Horizontal density required (rays/degree)
%         max_range: maximum Horizontal range including symmetry (m)
%         min_range: minimum Horizontal range including symmetry (m)
%         max_in_angle: maximum angle coverage of induvidual TUV (degrees)
%         min_in_angle: minimum angle coverage of induvidual TUV (degrees)
%       ----distribution for the specific scenario---
%         scn_max_angle (degrees)
%         scn_min_angle (degrees)
%         scn_max_density (rays/degree)
%         scn_min_density (rays/degree)
%         scn_max_range (m)
%         scn_min_range (m)
%       -----vertical field of view-----
%         vt_max_angle (degrees)
%         vt_min_angle (degrees)
%         vt_max_density (rays/degree)
%         vt_min_density (rays/degree)
%         vt_max_range (m)
%         vt_min_range (m)
    pose=zeros(4,2,6);%gives the position of 4 corners of the vehicle for 6 different cases
    x0=loc(1);
    y0=loc(2);%position of the laser in VUT0
    xmin=x(1);
    xmax=x(2);
    xpos=scenario(1);%forward or backward
    lane=scenario(2);%specifies which lane TUV is in
    y=1.5*b-dim1(2)/2;
    rays=5; %no.of rays mimimum to detect the object
    if xpos==1
        base=[xmin,y;xmin,0;xmin,-y;xmax,y;xmax,0;xmax,-y];%defines the centre position of 6 cases
        %order of this is very important
    elseif xpos==-1
        base=[-xmin,y;-xmin,0;-xmin,-y;-xmax,y;-xmax,0;-xmax,-y];
    end
    ang=zeros(6,4);
    euc_dist=zeros(6,4);
    for i=1:6 %calculates the co-ordinates of each corner
        pose(1,:,i)=base(i,:)+[-dim1(1)/2 -dim1(2)/2];
        pose(2,:,i)=base(i,:)+[-dim1(1)/2 dim1(2)/2];
        pose(3,:,i)=base(i,:)+[dim1(1)/2 dim1(2)/2];
        pose(4,:,i)=base(i,:)+[dim1(1)/2 -dim1(2)/2];
    end
    for l=1:6 %calculates the euclidian distance of each corner from the laser
        for m=1:4
            euc_dist(l,m)=sqrt((pose(m,1,l)-x0)^2+(pose(m,2,l)-y0)^2);
        end
    end
    for j=1:6%calculates the angle of each corner
        for k=1:4
            ang(j,k)=atan2d((pose(k,2,j)-y0),(pose(k,1,j)-x0));
            if ang(j,k)>=90 && ang(j,k)<=180
                ang(j,k)=180-ang(j,k);
            elseif ang(j,k)<-90 && ang(j,k)>=-180
                ang(j,k)=-(180+ang(j,k));
            end
        end
    end
    %----------------min and max sweep calculation-----------------
    batch1=ang(1:3,:); %first 3 cases, mimimum most position
    batch2=ang(4:6,:); %last 3 cases, maximum most position
    max_angle=2*max(batch1,[],'all'); %twice the angle for symmetry
    min_angle=2*max(batch2,[],'all');
    %---------------density calculation-------------------------------
    angle_diff=range(ang,2);
    max_in_angle=max(angle_diff); %the max angle coverage without symmetry and just the vehicle
    min_in_angle=min(angle_diff); %the max angle coverage without symmetry and just the vehicle
    min_density=rays/max(angle_diff); 
    max_density=rays/min(angle_diff);
    %---------------range calculation-------------------------------
    ang_extreme=zeros(4,4); %angles of the extreme cases only
    ang_extreme(1,:)=ang(1,:);
    ang_extreme(2:3,:)=ang(3:4,:);
    ang_extreme(4,:)=ang(6,:);
    euc_extreme=zeros(4,4); %distance of the extreme cases only
    euc_extreme(1,:)=euc_dist(1,:);
    euc_extreme(2:3,:)=euc_dist(3:4,:);
    euc_extreme(4,:)=euc_dist(6,:);
    abs_ex_ang=abs(ang_extreme);
    [~,I]=max(abs_ex_ang,[],2);
    [~,J]=min(abs_ex_ang,[],2);
    range_prime=zeros(4,2);
    for n=1:4
        range_prime(n,:)=[euc_extreme(n,I(n)),euc_extreme(n,J(n))];
    end
    max_range=max(range_prime,[],'all');
    min_range=min(euc_dist,[],'all');
    %------angle and range calculation for the specific scenario---------
    %calculates the distribution specific to that scenario
    abs_ang=abs(ang);
    if lane==1
        scn_max_angle=angle_diff(3);
        scn_min_angle=angle_diff(6);
        scn_max_density=rays/scn_min_angle;
        scn_min_density=rays/scn_max_angle;
        [~,K]=max(abs_ang(6,:));
        [~,L]=min(abs_ang(6,:));
        scn_range_prime=[euc_dist(6,K),euc_dist(6,L)];
        scn_max_range=max(scn_range_prime);
        scn_min_range=min(euc_dist(3,:));
    elseif lane==2
        scn_max_angle=angle_diff(2);
        scn_min_angle=angle_diff(5);
        scn_max_density=rays/scn_min_angle;
        scn_min_density=rays/scn_max_angle;
        [~,K]=max(abs_ang(5,:));
        [~,L]=min(abs_ang(5,:));
        scn_range_prime=[euc_dist(5,K),euc_dist(5,L)];
        scn_max_range=max(scn_range_prime);
        scn_min_range=min(euc_dist(2,:));
    elseif lane==3
        scn_max_angle=angle_diff(1);
        scn_min_angle=angle_diff(4);
        scn_max_density=rays/scn_min_angle;
        scn_min_density=rays/scn_max_angle;
        [~,K]=max(abs_ang(4,:));
        [~,L]=min(abs_ang(4,:));
        scn_range_prime=[euc_dist(4,K),euc_dist(4,L)];
        scn_max_range=max(scn_range_prime);
        scn_min_range=min(euc_dist(1,:));
    end
    %------------vertical field of view----------------------
    angz=zeros(1,2);
    angz1=zeros(1,2);
    angz2=zeros(1,2);
    angz1(1)=atan2d(dim0(3),xmin-dim1(1)/2); %calculates the bottom corner angle for min position
    angz1(2)=atan2d(dim0(3),xmax-dim1(1)/2); %calculates the bottom corner angle for max position
    angz2(1)=atan2d(abs(dim1(3)-dim0(3)),xmin-dim1(1)/2); %calculates the top corner angle for min position
    angz2(2)=atan2d(abs(dim1(3)-dim0(3)),xmax-dim1(1)/2); %calculates the top corner angle for max position
    if dim1(3)>dim0(3) 
        angz(1)=angz1(1)+angz2(1);
        angz(2)=angz1(2)+angz2(2);
        vt_max_angle=angz(1);
        vt_min_angle=angz(2);
    elseif dim1(3)<dim0(3)
        angz(1)=angz1(1)-angz2(1);
        angz(2)=angz1(2)-angz2(2);
        vt_max_angle=angz(1);
        vt_min_angle=angz(2);
    else
        vt_max_angle=angz1(1);
        vt_min_angle=angz1(2);
    end
    rayz=4;%no.of rays vertically
    vt_max_density=rayz/vt_min_angle;
    vt_min_density=rayz/vt_max_angle;
    vertical_dist=zeros(2,2);
    vertical_dist(1,1)=(xmin-dim1(1)/2)/cosd(angz1(1));
    vertical_dist(1,2)=(xmin-dim1(1)/2)/cosd(angz2(1));
    vertical_dist(2,1)=(xmax-dim1(1)/2)/cosd(angz1(2));
    vertical_dist(2,2)=(xmax-dim1(1)/2)/cosd(angz2(2));
    vt_max_range=max(vertical_dist(2,:));
    vt_min_range=min(vertical_dist(1,:));
    Distribution=[max_angle,min_angle,max_density,min_density,max_range,min_range,max_in_angle,min_in_angle,scn_max_angle,scn_min_angle,scn_max_density,scn_min_density,scn_max_range,scn_min_range,vt_max_angle,vt_min_angle,vt_max_density,vt_min_density,vt_max_range,vt_min_range];
end
    
    
    
    
    
         
                        
                