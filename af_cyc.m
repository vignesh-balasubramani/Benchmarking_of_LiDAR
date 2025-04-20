function af_cyc = af_cyc(L,W,H)
%function that calculates the area factor for cyclist given (length width and height)
    wb=1210; %wheelbasee in mm
    w_dia=700; %wheel diameter in mm
    w_w=150; %width of the wheel in mm
    frame_h=860; %frame height from ground in mm
    frame_base=280; %frame base from ground in mm
    handle_h=1180; %handle bar height from ground in mm
    pedal_h=495; %hieght of pedal in mm
    dummy_h=1800; %height of dummy in mm
    dummy_w=500; %width of dummy in mm
    head_h=260; %height of head in mm
    head_l=170; %length of head in mm
    head_w=170; %width of head in mm
    torso_l=235; %width of torso in mm
    hip_h=923; %height of hip in mm
    leg_w=176; %width of leg in mm
    %-----------calculating the areas-----------
    A_wheel_s=w_dia^2;
    A_frame_s=(frame_h-frame_base)*(wb-w_dia);
    A_torso_s=(dummy_h-head_h-hip_h)*torso_l;
    A_head_s=(head_h*head_l);
    A_hands_s=(handle_h-frame_h)*(wb-w_dia-torso_l);
    A_wheel_b=w_dia*w_w;
    A_leg_b=(frame_h-pedal_h)*leg_w;
    A_torso_b=(dummy_h-head_h-hip_h)*dummy_w;
    A_head_b=(head_h*head_w);
    side_area=H*L;
    act_side_area=(2*A_wheel_s+A_frame_s+A_torso_s+A_head_s+A_hands_s);
    af_side=act_side_area/side_area;
    back_area=H*W;
    act_back_area=(2*A_leg_b+A_wheel_b+A_torso_b+A_head_b);
    af_back=act_back_area/back_area;
    af_cyc=[af_side af_back];
end