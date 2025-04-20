function af_cyc = af_ad(L,W,H,a)
%function that calculates the area factor for pedestrian given (length width and height)
    if a==1 %dimensions of adult
        head_h=260; %height of head in mm
        head_l=170; %length of head in mm
        head_w=170; %width of head in mm
        torso_l=L; %width of torso in mm
        torso_h=1500-923; %height of torso in mm
        torso_w=W; %widdth of torso in mm
        leg_h=923; %height of hip in mm
        leg_w=176; %width
        leg_l=176; %height
    elseif a==0 %dimensions of child
        head_h=139; %height of head in mm
        head_l=150; %length of head in mm
        head_w=150; %width of head in mm
        torso_l=L; %width of torso in mm
        torso_h=920-607; %height of torso in mm
        torso_w=W;
        leg_h=607; %height of hip in mm
        leg_w=176;
        leg_l=176;
    end
    %-----------calculating the areas-----------
    A_torso_s=torso_h*torso_l;
    A_head_s=(head_h*head_l);
    A_leg_s=leg_h*leg_l;
    A_leg_b=leg_h*leg_w;
    A_torso_b=torso_h*torso_w;
    A_head_b=(head_h*head_w);
    side_area=H*L;
    act_side_area=(A_torso_s+A_head_s+2*A_leg_s);
    af_side=act_side_area/side_area;
    back_area=H*W;
    act_back_area=(2*A_leg_b+A_torso_b+A_head_b);
    af_back=act_back_area/back_area;
    af_cyc=[af_side af_back];
end