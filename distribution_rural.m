function Distribution=distribution_rural(x,b,dim0,dim1,loc,scenario)
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
%         max_density: maximum Horizontal density required (degree/ray)
%         min_density: minimum Horizontal density required (degree/ray)
%         max_range: maximum Horizontal range including symmetry (m)
%         min_range: minimum Horizontal range including symmetry (m)
%         max_in_angle: maximum angle coverage of induvidual TUV (degrees)
%         min_in_angle: minimum angle coverage of induvidual TUV (degrees)
%       -----vertical field of view-----
%         vt_max_angle (degrees)
%         vt_min_angle (degrees)
%         vt_max_density (degree/ray)
%         vt_min_density (degree/ray)
%         vt_max_range (m)
%         vt_min_range (m)
    pose=zeros(4,2,4);%gives the position of 4 corners of the vehicle for 4 different cases
    x0=loc(1);
    y0=loc(2);%position of the laser in VUT0
    xmin=x(1);
    xmax=x(2);
    xpos=scenario(1);%forward or backward
    lane=scenario(2);%specifies which lane TUV is in
    rays=5; %no.of rays mimimum to detect the object
    ang=zeros(4,4);
    euc_dist=zeros(4,4);
    if lane==1
        y=abs(0.5*b-dim1(2)/2);
        if xpos==1
            base=[xmin,y;xmin,-y;xmax,y;xmax,-y];%defines the centre position of 4 cases
                                                %order of this is very important
        elseif xpos==-1
            base=[-xmin,y;-xmin,-y;-xmax,y;-xmax,-y];
        end
    elseif lane==2
        y1=abs(0.5*b+dim1(2)/2);
        y2=abs(1.5*b-dim1(2)/2);
        if xpos==1
            base=[xmin,y2;xmin,y1;xmax,y2;xmax,y1];%defines the centre position of 4 cases
                                                %order of this is very important
        elseif xpos==-1
            base=[-xmin,y2;-xmin,y1;-xmax,y2;-xmax,y1];
        end
    end
    for i=1:4 %calculates the co-ordinates of each corner
        pose(1,:,i)=base(i,:)+[-dim1(1)/2 -dim1(2)/2];
        pose(2,:,i)=base(i,:)+[-dim1(1)/2 dim1(2)/2];
        pose(3,:,i)=base(i,:)+[dim1(1)/2 dim1(2)/2];
        pose(4,:,i)=base(i,:)+[dim1(1)/2 -dim1(2)/2];
    end
    for l=1:4 %calculates the euclidian distance of each corner from the laser
        for m=1:4
            euc_dist(l,m)=sqrt((pose(m,1,l)-x0)^2+(pose(m,2,l)-y0)^2);
        end
    end
    for j=1:4%calculates the angle of each corner
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
    batch1=abs(ang(1:2,:)); %first 2 cases, mimimum most position
    batch2=abs(ang(3:4,:)); %last 2 cases, maximum most position
    max_angle=2*max(batch1,[],'all'); %twice the angle for symmetry
    min_angle=2*max(batch2,[],'all');
    %---------------density calculation-------------------------------
    angle_diff=range(ang,2);
    max_in_angle=max(angle_diff); %the max angle coverage without symmetry and just the vehicle
    min_in_angle=min(angle_diff); %the max angle coverage without symmetry and just the vehicle
    min_density=max(angle_diff)/rays; 
    max_density=min(angle_diff)/rays;
    %---------------range calculation-------------------------------
%     ang_extreme=zeros(4,4); %angles of the extreme cases only
%     ang_extreme(1,:)=ang(1,:);
%     ang_extreme(2:3,:)=ang(3:4,:);
%     ang_extreme(4,:)=ang(6,:);
%     euc_extreme=zeros(4,4); %distance of the extreme cases only
%     euc_extreme(1,:)=euc_dist(1,:);
%     euc_extreme(2:3,:)=euc_dist(3:4,:);
%     euc_extreme(4,:)=euc_dist(6,:);
    abs_ex_ang=abs(ang);
    euc_extreme=euc_dist;
    [~,I]=max(abs_ex_ang,[],2);
    [~,J]=min(abs_ex_ang,[],2);
    range_prime=zeros(4,2);
    for n=1:4
        range_prime(n,:)=[euc_extreme(n,I(n)),euc_extreme(n,J(n))];
    end
    max_range=max(range_prime,[],'all');
    min_range=min(euc_dist,[],'all');
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
    vt_max_density=vt_min_angle/rayz;
    vt_min_density=vt_max_angle/rayz;
    vertical_dist=zeros(2,2);
    vertical_dist(1,1)=(xmin-dim1(1)/2)/cosd(angz1(1));
    vertical_dist(1,2)=(xmin-dim1(1)/2)/cosd(angz2(1));
    vertical_dist(2,1)=(xmax-dim1(1)/2)/cosd(angz1(2));
    vertical_dist(2,2)=(xmax-dim1(1)/2)/cosd(angz2(2));
    vt_max_range=max(vertical_dist(2,:));
    vt_min_range=max(vertical_dist(1,:));
%     Distribution=[max_angle,min_angle,max_density,min_density,max_range,min_range,max_in_angle,min_in_angle,scn_max_angle,scn_min_angle,scn_max_density,scn_min_density,scn_max_range,scn_min_range,vt_max_angle,vt_min_angle,vt_max_density,vt_min_density,vt_max_range,vt_min_range];
    Distribution=[max_angle,min_angle,max_density,min_density,max_range,min_range,max_in_angle,min_in_angle,vt_max_angle,vt_min_angle,vt_max_density,vt_min_density,vt_max_range,vt_min_range];
end
    
    
    
    
    
         
                        
                