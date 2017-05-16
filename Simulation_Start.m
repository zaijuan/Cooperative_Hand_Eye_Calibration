
clc
close all
clear all
% startup_rvc

% M number of data sets
M = 20;

% standard deviation of noise for Rotational part
std_devi_R = 0;
std_devi_R = pi/90;
%std_devi_R = pi/36;
% std_devi_R = 2*pi/45;
% std_devi_R = pi/18;
 
% standard deviation of noise for Translational part
std_devi_T = 0;
std_devi_T = 2;
%std_devi_T = 5;
% std_devi_t =8;
% std_devi_t =10;

% For camera, the noise is independent from above, here we set:
std_devi_Cam_T=0.4;
std_devi_Cam_R=0;
std_devi_Cam_R=pi/360;
 
H_w=[
    0     1      0 
    -1    0      0   
    0     0     -1    
    ];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Fixed SETTING UP For ROBOT-1 and Robot-2SYSTEM%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % camera-Robot calibration Value
        r1_H_c1=[
              1     0      0     30
              0     1      0     10
              0     0      1     30
              0     0      0     1
               ];
       c1_H_r1 = inv(r1_H_c1);
       r2_H_c2=[
              1     0      0     30
              0     1      0     10
              0     0      1     30
              0     0      0     1
               ];
      c2_H_r2 = inv(r2_H_c2);
      % Loop count for the number of times the process is performed
      LC = 30;
      R1_ROTERR_X_Tsai_AVER=0;
      R1_ROTERR_Y_Tsai_AVER=0;
      R1_ROTERR_Z_Tsai_AVER=0;
      R2_ROTERR_X_Tsai_AVER=0;
      R2_ROTERR_Y_Tsai_AVER=0;
      R2_ROTERR_Z_Tsai_AVER=0;
      R1_ROTERR_X_Coop_AVER=0;
      R1_ROTERR_Y_Coop_AVER=0;
      R1_ROTERR_Z_Coop_AVER=0;
      R2_ROTERR_X_Coop_AVER=0;
      R2_ROTERR_Y_Coop_AVER=0;
      R2_ROTERR_Z_Coop_AVER=0;
      R1_TRANSERR_Coop_AVER=zeros(3,1);
      R2_TRANSERR_Coop_AVER=zeros(3,1);
      R1_TRANSERR_Tsai_AVER=zeros(3,1);
      R2_TRANSERR_Tsai_AVER=zeros(3,1);
for counter = 1:LC
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% SETTING UP ROBOT-1 SYSTEM - R1 SETUP %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Not-Fixed transforms in R1 Setup %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % For all M, R1-setup positions calculate Camera (C1) w.r.t World Frame
    w_H_c1 = zeros(4,4,M);
    N_w_H_c1=zeros(4,4,M);
    N_c1_H_w = zeros(4,4,M);
    
   
    % Create M random poses of the R1 setup w.r.t World Frame  
    % The moving space for camera is X:-200,200   Y: -350,450   Z:200,700
    w_H_c1(:,:,1)=transl([randi([-200, 200],1,1); randi([-350, 450],1,1); randi([200, 700],1,1)]);
    % the coordinate of the center of the board is:(45,50)
    xdif=45-w_H_c1(1,4,1);
    ydif=50-w_H_c1(2,4,1);
    x_rad=atan(ydif/w_H_c1(3,4,1)) +randi([-200, 200])*pi/600;  
    y_rad=atan(xdif/w_H_c1(3,4,1)) +randi([-200, 200])*pi/600;
    z_rad= randi([-10, 10])*pi/60; %generate a random range between: -pi/6 pi/6
    X_rotat=Rotat_X(x_rad);
    Y_rotat=Rotat_Y(y_rad);
    Z_rotat=Rotat_Z(z_rad);
    
    w_H_c1(1:3,1:3,1)=Z_rotat * Y_rotat * X_rotat * H_w;
    
    %add noise to w_H_c1
     measure = w_H_c1(:,:,1);
     measure(1:3,4) =measure(1:3,4)+std_devi_Cam_T*randn(3,1);
     x_rad=x_rad+std_devi_Cam_R*randn(1,1);
     y_rad=y_rad+std_devi_Cam_R*randn(1,1);
     z_rad=z_rad+std_devi_Cam_R*randn(1,1);
     X_rotat=Rotat_X(x_rad);
     Y_rotat=Rotat_Y(y_rad);
     Z_rotat=Rotat_Z(z_rad);
     measure(1:3,1:3)=Z_rotat * Y_rotat * X_rotat * H_w;
     N_w_H_c1(:,:,1) = measure;
     
      %add noise to the first measurement of w_H_r1
     w_H_r1(:,:,1) = w_H_c1(:,:,1) * c1_H_r1;
     r1_H_w(:,:,1) = inv(w_H_r1(:,:,1));
     measure = w_H_c1(:,:,1);
     measure(1:3,4) =w_H_r1(1:3,4,1)+std_devi_T*randn(3,1);
     x_rad_new=x_rad+std_devi_R*randn(1,1);
     y_rad_new=y_rad+std_devi_R*randn(1,1);
     z_rad_new=z_rad+std_devi_R*randn(1,1);
     X_rotat=Rotat_X(x_rad_new);
     Y_rotat=Rotat_Y(y_rad_new);
     Z_rotat=Rotat_Z(z_rad_new);
     measure(1:3,1:3)=Z_rotat * Y_rotat * X_rotat*H_w;
     N_w_H_r1(:,:,1) = measure;
    
    for i=2:M
        %determin the next position where the camera would be, ensure
        %enough translation in x,y,z direction
      if(w_H_c1(1,4,i-1)+200>=60)   xmin=60; else xmin=w_H_c1(1,4,i-1)+200; 
      end
      if((200-w_H_c1(1,4,i-1)>=60)) xmax=60; else xmax=200-w_H_c1(1,4,i-1); 
      end
      if(w_H_c1(2,4,i-1)+350>=60)   ymin=60; else ymin=w_H_c1(2,4,i-1)+350;  
      end
      if((450-w_H_c1(2,4,i-1)>=60)) ymax=60; else ymax=450-w_H_c1(2,4,i-1);
      end
      if(w_H_c1(3,4,i-1)-200>=100)   zmin=100; else zmin=w_H_c1(3,4,i-1)-200;
      end
      if((700-w_H_c1(3,4,i-1)>=100)) zmax=100; else zmax=700-w_H_c1(3,4,i-1);
      end
      w_H_c1(:,:,i)=transl([randi([w_H_c1(1,4,i-1)-xmin, w_H_c1(1,4,i-1)+xmax],1,1); ...
                            randi([w_H_c1(2,4,i-1)-ymin, w_H_c1(2,4,i-1)+ymax],1,1); ...
                            randi([w_H_c1(3,4,i-1)-zmin, w_H_c1(3,4,i-1)+zmax],1,1)]);
                        
    %generate the rotation matrix.
    xdif=45-w_H_c1(1,4,i);
    ydif=50-w_H_c1(2,4,i);
    x_rad=atan(ydif/w_H_c1(3,4,i)) +randi([-200, 200])*pi/600;  
    y_rad=atan(xdif/w_H_c1(3,4,i)) +randi([-200, 200])*pi/600;
    z_rad= randi([-10, 10])*pi/60; %generate a random range between: -pi/6 pi/6
    X_rotat=Rotat_X(x_rad);
    Y_rotat=Rotat_Y(y_rad);
    Z_rotat=Rotat_Z(z_rad);
    w_H_c1(1:3,1:3,i)=Z_rotat * Y_rotat * X_rotat * H_w;
    
     %add noise to w_H_c1  
     measure = w_H_c1(:,:,i);
     measure(1:3,4) =measure(1:3,4)+std_devi_Cam_T*randn(3,1);
     x_rad_new=x_rad+std_devi_Cam_R*randn(1,1);
     y_rad_new=y_rad+std_devi_Cam_R*randn(1,1);
     z_rad_new=z_rad+std_devi_Cam_R*randn(1,1);
     X_rotat=Rotat_X(x_rad_new);
     Y_rotat=Rotat_Y(y_rad_new);
     Z_rotat=Rotat_Z(z_rad_new);
     measure(1:3,1:3)=Z_rotat * Y_rotat * X_rotat*H_w;
     N_w_H_c1(:,:,i) = measure;
     N_c1_H_w(:,:,i) = inv( N_w_H_c1(:,:,i) ); % use the noise-corrupted value of the camera
    
     %add noise to w_H_r1
     w_H_r1(:,:,i) = w_H_c1(:,:,i) * c1_H_r1;
     r1_H_w(:,:,i) = inv(w_H_r1(:,:,i));
     measure = w_H_c1(:,:,i);
     measure(1:3,4) =w_H_r1(1:3,4,i)+std_devi_T*randn(3,1);
     x_rad_new=x_rad+std_devi_R*randn(1,1);
     y_rad_new=y_rad+std_devi_R*randn(1,1);
     z_rad_new=z_rad+std_devi_R*randn(1,1);
     X_rotat=Rotat_X(x_rad_new);
     Y_rotat=Rotat_Y(y_rad_new);
     Z_rotat=Rotat_Z(z_rad_new);
     measure(1:3,1:3)=Z_rotat * Y_rotat * X_rotat*H_w;
     N_w_H_r1(:,:,i) = measure;
     
    end
           
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% SETTING UP ROBOT-2 SYSTEM - R2 SETUP %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Not-Fixed transforms in R2 Setup %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % For all M R1-setup positions calculate Camera  positions w.r.t World Frame
    w_H_c2 = zeros(4,4,M);
    N_w_H_c2 = zeros(4,4,M);
    N_c2_H_w = zeros(4,4,M);
    
    % Create M random poses of the R1 setup w.r.t World Frame
    % The moving space for camera is X:-200,200   Y: -350,450   Z:200,700
    w_H_c2(:,:,1)=transl([randi([-200, 200],1,1); randi([-350, 450],1,1); randi([200, 700],1,1)]);
    % the coordinate of the center of the board is:(45,50)
    xdif=45-w_H_c2(1,4,1);
    ydif=50-w_H_c2(2,4,1);
    x_rad=atan(ydif/w_H_c2(3,4,1))+randi([-200, 200])*pi/600;
    y_rad=atan(xdif/w_H_c2(3,4,1))+randi([-200, 200])*pi/600;
    X_rotat=Rotat_X(x_rad);
    Y_rotat=Rotat_Y(y_rad);
    z_rad=randi([-10, 10])*pi/60; %generate a random range between: -pi/6 pi/6
    Z_rotat=Rotat_Z(z_rad);
    w_H_c2(1:3,1:3,1)=Z_rotat * Y_rotat * X_rotat * H_w;
     
    %add noise to camera 2's first measurement data
     measure = w_H_c2(:,:,1);
     measure(1:3,4) =measure(1:3,4)+std_devi_Cam_T*randn(3,1);
     x_rad=x_rad+std_devi_Cam_R*randn(1,1);
     y_rad=y_rad+std_devi_Cam_R*randn(1,1);
     z_rad=z_rad+std_devi_Cam_R*randn(1,1);
     X_rotat=Rotat_X(x_rad);
     Y_rotat=Rotat_Y(y_rad);
     Z_rotat=Rotat_Z(z_rad);
     measure(1:3,1:3) =Z_rotat * Y_rotat * X_rotat * H_w;
     N_w_H_c2(:,:,1) = measure;
     
     %add noise to the first measurement of w_H_r1
     w_H_r2(:,:,1) = w_H_c2(:,:,1) * c2_H_r2;
     r2_H_w(:,:,1) = inv(w_H_r2(:,:,1));
     measure = w_H_c2(:,:,1);
     measure(1:3,4) =w_H_r2(1:3,4,1)+std_devi_T*randn(3,1);
     x_rad_new=x_rad+std_devi_R*randn(1,1);
     y_rad_new=y_rad+std_devi_R*randn(1,1);
     z_rad_new=z_rad+std_devi_R*randn(1,1);
     X_rotat=Rotat_X(x_rad_new);
     Y_rotat=Rotat_Y(y_rad_new);
     Z_rotat=Rotat_Z(z_rad_new);
     measure(1:3,1:3)=Z_rotat * Y_rotat * X_rotat*H_w;
     N_w_H_r2(:,:,1) = measure;
    
    for i=2:M
        %determin the next position where the camera would be, ensure
        %enough translation in x,y,z direction
      if(w_H_c2(1,4,i-1)+200>=60)   xmin=60; else xmin=w_H_c2(1,4,i-1)+200; 
      end
      if((200-w_H_c2(1,4,i-1)>=60)) xmax=60; else xmax=200-w_H_c2(1,4,i-1); 
      end
      if(w_H_c2(2,4,i-1)+350>=60)   ymin=60; else ymin=w_H_c2(2,4,i-1)+350;  
      end
      if((450-w_H_c2(2,4,i-1)>=60)) ymax=60; else ymax=450-w_H_c2(2,4,i-1);
      end
      if(w_H_c2(3,4,i-1)-200>=100)   zmin=100; else zmin=w_H_c2(3,4,i-1)-200;
      end
      if((700-w_H_c2(3,4,i-1)>=100)) zmax=100; else zmax=700-w_H_c2(3,4,i-1);
      end
      w_H_c2(:,:,i)=transl([randi([w_H_c2(1,4,i-1)-xmin, w_H_c2(1,4,i-1)+xmax],1,1); ...
                            randi([w_H_c2(2,4,i-1)-ymin, w_H_c2(2,4,i-1)+ymax],1,1); ...
                            randi([w_H_c2(3,4,i-1)-zmin, w_H_c2(3,4,i-1)+zmax],1,1)]);
                        
    %%%generate the rotation matrix.
    xdif=45-w_H_c2(1,4,i);
    ydif=50-w_H_c2(2,4,i);
    x_rad=atan(ydif/w_H_c2(3,4,i))+randi([-200, 200])*pi/600;
    y_rad=atan(xdif/w_H_c2(3,4,i))+randi([-200, 200])*pi/600;
    X_rotat=Rotat_X(x_rad);
    Y_rotat=Rotat_Y(y_rad);
    z_rad=randi([-10, 10])*pi/60; %generate a random range between: -pi/6 pi/6
    Z_rotat=Rotat_Z(z_rad);
    w_H_c2(1:3,1:3,i)=Z_rotat * Y_rotat * X_rotat * H_w;
    
     %add noise to camera 2
     measure = w_H_c2(:,:,i);
     measure(1:3,4) =measure(1:3,4)+std_devi_Cam_T*randn(3,1);
     x_rad=x_rad+std_devi_Cam_R*randn(1,1);
     y_rad=y_rad+std_devi_Cam_R*randn(1,1);
     z_rad=z_rad+std_devi_Cam_R*randn(1,1);
     X_rotat=Rotat_X(x_rad);
     Y_rotat=Rotat_Y(y_rad);
     Z_rotat=Rotat_Z(z_rad);
     measure(1:3,1:3)=Z_rotat * Y_rotat * X_rotat * H_w;
     N_w_H_c2(:,:,i) = measure;
     
     %add noise to w_H_r1
     w_H_r2(:,:,i) = w_H_c2(:,:,i) * c2_H_r2;
     r2_H_w(:,:,i) = inv(w_H_r2(:,:,i));
     measure = w_H_c2(:,:,i);
     measure(1:3,4) =w_H_r2(1:3,4,i)+std_devi_T*randn(3,1);
     x_rad_new=x_rad+std_devi_R*randn(1,1);
     y_rad_new=y_rad+std_devi_R*randn(1,1);
     z_rad_new=z_rad+std_devi_R*randn(1,1);
     X_rotat=Rotat_X(x_rad_new);
     Y_rotat=Rotat_Y(y_rad_new);
     Z_rotat=Rotat_Z(z_rad_new);
     measure(1:3,1:3)=Z_rotat * Y_rotat * X_rotat*H_w;
     N_w_H_r2(:,:,i) = measure;
        
    end
       
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% TRANSFORMATIONS BASED ON R1 AND R2 SETUPS DATA %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = 1:M-1
                       
        N_c2t1_H_c1t1(:,:,i) = N_c2_H_w(:,:,i)*N_w_H_c1(:,:,i);           %this is the A matrix in the function AX'BX=Y'CYD
            
        N_c2t2_H_c1t2(:,:,i) = N_c2_H_w(:,:,i+1)*N_w_H_c1(:,:,i+1);          % this is the D matrix in the function AX'BX=Y'CYD       
       
        %the noise of the relationship between the cameras(either of different time stamp or different cameras) is fixed,
        %independent of the noise added to the odometry data. :)        
        N_r1t1_H_r1t2(:,:,i) = inv( N_w_H_r1(:,:,i) ) * N_w_H_r1(:,:,i+1);    % B matrix in the function AX'BX=Y'CYD  
        N_r2t1_H_r2t2(:,:,i) = inv( N_w_H_r2(:,:,i) ) * N_w_H_r2(:,:,i+1);    % C matrix in the function AX'BX=Y'CYD 
                
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Initialization Values of fixed unknowns for Co-operative Calibration %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%  For robot 1  %%%%%%%%%%%%%    
        measure=r1_H_c1;
        measure(1:3,4) =measure(1:3,4)+std_devi_T*randn(3,1);
        [wgn_rot_x,wgn_rot_y,wgn_rot_z]=decom_rot(measure(1:3,1:3));
        wgn_rot_x=wgn_rot_x+std_devi_R*randn(1,1);
        wgn_rot_y=wgn_rot_y+std_devi_R*randn(1,1);
        wgn_rot_z=wgn_rot_z+std_devi_R*randn(1,1);
        measure(1:3,1:3)=com_rot(wgn_rot_x,wgn_rot_y,wgn_rot_z);
        r1_H_c1_initialVal = measure;
        
    %%%%%%%%%%%%%  For robot 2  %%%%%%%%%%%%%          
        measure=r2_H_c2;
        measure(1:3,4) =measure(1:3,4)+std_devi_T*randn(3,1);
        [wgn_rot_x,wgn_rot_y,wgn_rot_z]=decom_rot(measure(1:3,1:3));
        wgn_rot_x=wgn_rot_x+std_devi_R*randn(1,1);
        wgn_rot_y=wgn_rot_y+std_devi_R*randn(1,1);
        wgn_rot_z=wgn_rot_z+std_devi_R*randn(1,1);
        measure(1:3,1:3)=com_rot(wgn_rot_x,wgn_rot_y,wgn_rot_z);
        r2_H_c2_initialVal = measure;
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% C-R for R1 and C-R for R2 Setup %%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [ROTAT1_TRUE_X,ROTAT1_TRUE_Y,ROTAT1_TRUE_Z]=decom_rot(r1_H_c1(1:3,1:3));
    [ROTAT2_TRUE_X,ROTAT2_TRUE_Y,ROTAT2_TRUE_Z]=decom_rot(r2_H_c2(1:3,1:3));
  
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%% C-R for R1 Setup using Tsai-Lenz %%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 r1_H_c1_Cal_Tsai = handEye_New(N_w_H_r1, N_w_H_c1);    % N_c1t1_H_c1t2 is the A in function AX=XB.
          TRANSR1_ERR=abs(r1_H_c1_Cal_Tsai(1:3,4)-r1_H_c1(1:3,4));
          [ROTAT1_CAL_X,ROTAT1_CAL_Y,ROTAT1_CAL_Z]=decom_rot(r1_H_c1_Cal_Tsai(1:3,1:3));
          ROTAT1_ERR_X_Tsai=abs(ROTAT1_TRUE_X-ROTAT1_CAL_X);
          ROTAT1_ERR_Y_Tsai=abs(ROTAT1_TRUE_Y-ROTAT1_CAL_Y);
          ROTAT1_ERR_Z_Tsai=abs(ROTAT1_TRUE_Z-ROTAT1_CAL_Z);
          disp('Tsai method Robot 1 Translation error = ');
          disp(TRANSR1_ERR);
          disp('Tsai method Robot 1 Rotation error = ');
          disp([ ROTAT1_ERR_X_Tsai ROTAT1_ERR_Y_Tsai ROTAT1_ERR_Z_Tsai]);
          R1_ROTERR_X_Tsai_AVER=R1_ROTERR_X_Tsai_AVER+ROTAT1_ERR_X_Tsai;
          R1_ROTERR_Y_Tsai_AVER=R1_ROTERR_Y_Tsai_AVER+ROTAT1_ERR_Y_Tsai;
          R1_ROTERR_Z_Tsai_AVER=R1_ROTERR_Z_Tsai_AVER+ROTAT1_ERR_Z_Tsai;
         
          R1_TRANSERR_Tsai_AVER=R1_TRANSERR_Tsai_AVER+TRANSR1_ERR;
     
    
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%% C-R for R2 Setup using Tsai-Lenz %%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  r2_H_c2_Cal_Tsai = handEye_New(N_w_H_r2, N_w_H_c2);  %  N_c2t1_H_c2t2 is the A in function AX=XB.
          TRANSR2_ERR=abs(r2_H_c2_Cal_Tsai(1:3,4)-r2_H_c2(1:3,4));
          [ROTAT2_CAL_X,ROTAT2_CAL_Y,ROTAT2_CAL_Z]=decom_rot(r2_H_c2_Cal_Tsai(1:3,1:3));
          ROTAT2_ERR_X_Tsai=abs(ROTAT2_TRUE_X-ROTAT2_CAL_X);
          ROTAT2_ERR_Y_Tsai=abs(ROTAT2_TRUE_Y-ROTAT2_CAL_Y);
          ROTAT2_ERR_Z_Tsai=abs(ROTAT2_TRUE_Z-ROTAT2_CAL_Z);
          disp('Tsai method Robot 2 Translation error = ');
          disp(TRANSR2_ERR);
          disp('Tsai method Robot 2 Rotation error = ');
          disp([ ROTAT2_ERR_X_Tsai ROTAT2_ERR_Y_Tsai ROTAT2_ERR_Z_Tsai]);
          R2_ROTERR_X_Tsai_AVER=R2_ROTERR_X_Tsai_AVER+ROTAT2_ERR_X_Tsai;
          R2_ROTERR_Y_Tsai_AVER=R2_ROTERR_Y_Tsai_AVER+ROTAT2_ERR_Y_Tsai;
          R2_ROTERR_Z_Tsai_AVER=R2_ROTERR_Z_Tsai_AVER+ROTAT2_ERR_Z_Tsai;
          
          R2_TRANSERR_Tsai_AVER=R2_TRANSERR_Tsai_AVER+TRANSR2_ERR;
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%% C-R for R1 and R2 Setups simultaneously using the cooperative method%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   [r1_H_c1_Cal_Coop, r2_H_c2_Cal_Coop] = U2_COOP_R1R2(N_c2t1_H_c1t1, N_r1t1_H_r1t2, N_r2t1_H_r2t2, N_c2t2_H_c1t2, ...
                                                   r1_H_c1_initialVal, r2_H_c2_initialVal);
          TRANSR1_ERR=abs(r1_H_c1_Cal_Coop(1:3,4)-r1_H_c1(1:3,4));
          [ROTAT1_CAL_X,ROTAT1_CAL_Y,ROTAT1_CAL_Z]=decom_rot(r1_H_c1_Cal_Coop(1:3,1:3));
          ROTAT1_ERR_X_Coop=abs(ROTAT1_TRUE_X-ROTAT1_CAL_X);
          ROTAT1_ERR_Y_Coop=abs(ROTAT1_TRUE_Y-ROTAT1_CAL_Y);
          ROTAT1_ERR_Z_Coop=abs(ROTAT1_TRUE_Z-ROTAT1_CAL_Z);
          disp('Cooperative method: Robot 1 Translation error = ');
          disp(TRANSR1_ERR);
          disp('Cooperative method:Robot  1 Rotation error = ');
          disp([ ROTAT1_ERR_X_Coop ROTAT1_ERR_Y_Coop ROTAT1_ERR_Z_Coop]);
          
          TRANSR2_ERR=abs(r2_H_c2_Cal_Coop(1:3,4)-r2_H_c2(1:3,4));
          [ROTAT2_CAL_X,ROTAT2_CAL_Y,ROTAT2_CAL_Z]=decom_rot(r2_H_c2_Cal_Coop(1:3,1:3));
          ROTAT2_ERR_X_Coop=abs(ROTAT2_TRUE_X-ROTAT2_CAL_X);
          ROTAT2_ERR_Y_Coop=abs(ROTAT2_TRUE_Y-ROTAT2_CAL_Y);
          ROTAT2_ERR_Z_Coop=abs(ROTAT2_TRUE_Z-ROTAT2_CAL_Z);
          disp('Cooperative method: Robot 2 Translation error = ');
          disp(TRANSR2_ERR);
          disp('Cooperative method: Robot 2 Rotation error = ');
          disp([ ROTAT2_ERR_X_Coop ROTAT2_ERR_Y_Coop ROTAT2_ERR_Z_Coop]);
          
           R1_ROTERR_X_Coop_AVER=R1_ROTERR_X_Coop_AVER+ROTAT1_ERR_X_Coop;
           R1_ROTERR_Y_Coop_AVER=R1_ROTERR_Y_Coop_AVER+ROTAT1_ERR_Y_Coop;
           R1_ROTERR_Z_Coop_AVER=R1_ROTERR_Z_Coop_AVER+ROTAT1_ERR_Z_Coop;
           R2_ROTERR_X_Coop_AVER=R2_ROTERR_X_Coop_AVER+ROTAT2_ERR_X_Coop;
           R2_ROTERR_Y_Coop_AVER=R2_ROTERR_Y_Coop_AVER+ROTAT2_ERR_Y_Coop;
           R2_ROTERR_Z_Coop_AVER=R2_ROTERR_Z_Coop_AVER+ROTAT2_ERR_Z_Coop;
           
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Using Tsai-Lenz to solve translation 
 
          r1_H_c1_Cal_Tsai = handEye_New_New(N_w_H_r1, N_w_H_c1,r1_H_c1_Cal_Coop);    % N_c1t1_H_c1t2 is the A in function AX=XB.
          TRANSR1_ERR=abs(r1_H_c1_Cal_Tsai(1:3,4)-r1_H_c1(1:3,4));
          disp('COOP_Tsai method Robot 1 Translation error = ');
          disp(TRANSR1_ERR);
      
          
          r2_H_c2_Cal_Tsai = handEye_New_New(N_w_H_r2, N_w_H_c2, r2_H_c2_Cal_Coop);  %  N_c2t1_H_c2t2 is the A in function AX=XB.
          TRANSR2_ERR=abs(r2_H_c2_Cal_Tsai(1:3,4)-r2_H_c2(1:3,4));
          disp('COOP_Tsai method Robot 2 Translation error = ');
          disp(TRANSR2_ERR);
              
           R2_TRANSERR_Coop_AVER=R2_TRANSERR_Coop_AVER+TRANSR2_ERR;
           R1_TRANSERR_Coop_AVER=R1_TRANSERR_Coop_AVER+TRANSR1_ERR;
         
              
    counter
    
end
    disp('      M    std_devi_R   std_devi_T  std_devi_Cam_R  std_devi_Cam_T')
    disp([M   std_devi_R     std_devi_T     std_devi_Cam_R    std_devi_Cam_T])
      R1_ROTERR_X_Tsai_AVER=(R1_ROTERR_X_Tsai_AVER/LC)*180/pi;
      R1_ROTERR_Y_Tsai_AVER=(R1_ROTERR_Y_Tsai_AVER/LC)*180/pi;
      R1_ROTERR_Z_Tsai_AVER=(R1_ROTERR_Z_Tsai_AVER/LC)*180/pi;
      R1_ALLROTERR_Tsai_AVER=R1_ROTERR_X_Tsai_AVER+R1_ROTERR_Y_Tsai_AVER+R1_ROTERR_Z_Tsai_AVER
      R1_ROTERR_X_Coop_AVER=(R1_ROTERR_X_Coop_AVER/LC)*180/pi;
      R1_ROTERR_Y_Coop_AVER=(R1_ROTERR_Y_Coop_AVER/LC)*180/pi;
      R1_ROTERR_Z_Coop_AVER=(R1_ROTERR_Z_Coop_AVER/LC)*180/pi;
      R1_ALLROTERR_COOP_AVER=R1_ROTERR_X_Coop_AVER+R1_ROTERR_Y_Coop_AVER+R1_ROTERR_Z_Coop_AVER
      R2_ROTERR_X_Tsai_AVER=(R2_ROTERR_X_Tsai_AVER/LC)*180/pi;
      R2_ROTERR_Y_Tsai_AVER=(R2_ROTERR_Y_Tsai_AVER/LC)*180/pi;
      R2_ROTERR_Z_Tsai_AVER=(R2_ROTERR_Z_Tsai_AVER/LC)*180/pi;
      R2_ALLROTERR_Tsai_AVER=R2_ROTERR_X_Tsai_AVER+R2_ROTERR_Y_Tsai_AVER+R2_ROTERR_Z_Tsai_AVER
      R2_ROTERR_X_Coop_AVER=(R2_ROTERR_X_Coop_AVER/LC)*180/pi;
      R2_ROTERR_Y_Coop_AVER=(R2_ROTERR_Y_Coop_AVER/LC)*180/pi;
      R2_ROTERR_Z_Coop_AVER=(R2_ROTERR_Z_Coop_AVER/LC)*180/pi;
      R2_ALLROTERR_COOP_AVER=R2_ROTERR_X_Coop_AVER+R2_ROTERR_Y_Coop_AVER+R2_ROTERR_Z_Coop_AVER
      R1_TRANSERR_Tsai_AVER  
      R2_TRANSERR_Tsai_AVER
      R1_TRANSERR_Coop_AVER  
      R2_TRANSERR_Coop_AVER
% Code End
cend = 0;

