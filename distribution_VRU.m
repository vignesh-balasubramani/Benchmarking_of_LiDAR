function Distribution=distribution_VRU(dim0,dim1,b1,b2,zebra,gap,loc)
    pose=zeros(4,2,9);
    ang=zeros(9,4);
    euc_dist=zeros(9,4);
    rays=5; %no.of rays mimimum to detect the object
    x0=loc(1);
    y0=loc(2);
    x_VRU=b2+gap+zebra/2;
    y_VRU=b1+gap+zebra/2;
    VRU=[-x_VRU,-y_VRU ; -x_VRU,y_VRU ; x_VRU,y_VRU ; x_VRU,-y_VRU ; -x_VRU y0];
    for a=1:5
        pose(1,:,a)=VRU(a,:)+[-dim1(2)/2,-dim1(1)/2];
        pose(2,:,a)=VRU(a,:)+[-dim1(2)/2,dim1(1)/2];
        pose(3,:,a)=VRU(a,:)+[dim1(2)/2,dim1(1)/2];
        pose(4,:,a)=VRU(a,:)+[dim1(2)/2,-dim1(1)/2];
    end
    b=1;
    for c=6:9
        pose(1,:,c)=VRU(b,:)+[-dim1(1)/2,-dim1(2)/2];
        pose(2,:,c)=VRU(b,:)+[-dim1(1)/2,dim1(2)/2];
        pose(3,:,c)=VRU(b,:)+[dim1(1)/2,dim1(2)/2];
        pose(4,:,c)=VRU(b,:)+[dim1(1)/2,-dim1(2)/2];
        b=b+1;
    end
    for d=1:9 %calculates the euclidian distance of each corner from the laser
        for e=1:4
            euc_dist(d,e)=sqrt((pose(e,1,d)-x0)^2+(pose(e,2,d)-y0)^2);
        end
    end
    for f=1:9%calculates the angle of each corner
        for g=1:4
            ang(f,g)=atan2d((pose(g,2,f)-y0),(pose(g,1,f)-x0));
            if ang(f,g)>=90 && ang(f,g)<=180
                ang(f,g)=180-ang(f,g);
            elseif ang(f,g)<-90 && ang(f,g)>=-180
                ang(f,g)=-(180+ang(f,g));
            end
        end
    end
    %---------------density calculation-------------------------------
    angle_diff=range(ang,2);
    max_in_angle=max(angle_diff); %the max angle coverage without symmetry and just the vehicle
    min_in_angle=min(angle_diff); %the max angle coverage without symmetry and just the vehicle
    min_density=1/(max(angle_diff)/rays); 
    max_density=1/(min(angle_diff)/rays);
    %---------------range calculation-------------------------------
    ang_extreme=zeros(8,4); %angles of the extreme cases only
    ang_extreme(1:4,:)=ang(1:4,:);
    ang_extreme(5:8,:)=ang(6:9,:);
    euc_extreme=zeros(8,4); %distance of the extreme cases only
    euc_extreme(1:4,:)=euc_dist(1:4,:);
    euc_extreme(5:8,:)=euc_dist(6:9,:);
    abs_ex_ang=abs(ang_extreme);
    [~,I]=max(abs_ex_ang,[],2);
    [~,J]=min(abs_ex_ang,[],2);
    range_prime=zeros(8,2);
    for n=1:8
        range_prime(n,:)=[euc_extreme(n,I(n)),euc_extreme(n,J(n))];
    end
    max_range=max(range_prime,[],'all');
    min_range=min(euc_dist,[],'all');
    %------------vertical field of view----------------------
    angz=zeros(1,2);
    angz1=zeros(1,2);
    angz2=zeros(1,2);
    xmin=zebra/2;
    xmax=(b2+gap)*2+(zebra)*1.5;
    angz1(1)=atan2d(dim0(3),xmin-dim1(2)/2); %calculates the bottom corner angle for min position
    angz1(2)=atan2d(dim0(3),xmax-dim1(2)/2); %calculates the bottom corner angle for max position
    angz2(1)=atan2d(abs(dim1(3)-dim0(3)),xmin-dim1(2)/2); %calculates the top corner angle for min position
    angz2(2)=atan2d(abs(dim1(3)-dim0(3)),xmax-dim1(2)/2); %calculates the top corner angle for max position
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
    vt_max_density=1/(vt_min_angle/rayz);
    vt_min_density=1/(vt_max_angle/rayz);
    vertical_dist=zeros(2,2);
    vertical_dist(1,1)=(xmin-dim1(2)/2)/cosd(angz1(1));
    vertical_dist(1,2)=(xmin-dim1(2)/2)/cosd(angz2(1));
    vertical_dist(2,1)=(xmax-dim1(2)/2)/cosd(angz1(2));
    vertical_dist(2,2)=(xmax-dim1(2)/2)/cosd(angz2(2));
    vt_max_range=max(vertical_dist(2,:));
    vt_min_range=max(vertical_dist(1,:));
    Distribution=[max_in_angle,min_in_angle,max_density,min_density,max_range,min_range,vt_max_angle,vt_min_angle,vt_max_density,vt_min_density,vt_max_range,vt_min_range];
end