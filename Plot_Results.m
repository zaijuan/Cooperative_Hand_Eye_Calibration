
clc; clear all

Num_Files = 10;

for num = 1: Num_Files
    filename = ['CR_000p5_Noise_', num2str(num) ,'0_Frames_100_Trials.mat'];
%     filename = ['CR_001p0_Noise_', num2str(num) ,'0_Frames_100_Trials.mat'];
%     filename = ['CR_005p0_Noise_', num2str(num) ,'0_Frames_100_Trials.mat'];
%     filename = ['CR_010p0_Noise_', num2str(num) ,'0_Frames_100_Trials.mat'];
    load(filename)
    
    True_CR1 = TRUE_r1_H_c1;
    True_CR2 = TRUE_r2_H_c2;
    
    Tsai_CR1 = real(U1_CR_R1_r1_H_c1);
    Tsai_CR2 = real(U1_CR_R2_r2_H_c2);
    
    Algo_CR1 = real(U2_CR_R1R2_r1_H_c1);
    Algo_CR2 = real(U2_CR_R1R2_r2_H_c2);
    
    TRIALS = size(True_CR1,3);
    
    Tsai_CR1_Rot_Diff = 0;
    Tsai_CR2_Rot_Diff = 0;
    Algo_CR1_Rot_Diff = 0;
    Algo_CR2_Rot_Diff = 0;
    
    Tsai_CR1_Tra_Diff = 0;
    Tsai_CR2_Tra_Diff = 0;
    Algo_CR1_Tra_Diff = 0;
    Algo_CR2_Tra_Diff = 0;
    
    for i = 1:TRIALS
        % Rotation Vector Absolute Difference
        Tsai_CR1_Rot_Diff = Tsai_CR1_Rot_Diff + abs( vrrotmat2vec(True_CR1(1:3,1:3,i)) - vrrotmat2vec(Tsai_CR1(1:3,1:3,i)) );
        
        Tsai_CR2_Rot_Diff = Tsai_CR2_Rot_Diff + abs( vrrotmat2vec(True_CR2(1:3,1:3,i)) - vrrotmat2vec(Tsai_CR2(1:3,1:3,i)) );
        
        Algo_CR1_Rot_Diff = Algo_CR1_Rot_Diff + abs( vrrotmat2vec(True_CR1(1:3,1:3,i)) - vrrotmat2vec(Algo_CR1(1:3,1:3,i)) );
        
        Algo_CR2_Rot_Diff = Algo_CR2_Rot_Diff + abs( vrrotmat2vec(True_CR2(1:3,1:3,i)) - vrrotmat2vec(Algo_CR2(1:3,1:3,i)) );
        
        % Translation Vector Absolute Difference
        Tsai_CR1_Tra_Diff = Tsai_CR1_Tra_Diff + abs( True_CR1(1:3,4,i) - Tsai_CR1(1:3,4,i) );
        
        Tsai_CR2_Tra_Diff = Tsai_CR2_Tra_Diff + abs( True_CR2(1:3,4,i) - Tsai_CR2(1:3,4,i) );
        
        Algo_CR1_Tra_Diff = Algo_CR1_Tra_Diff + abs( True_CR1(1:3,4,i) - Algo_CR1(1:3,4,i) );
        
        Algo_CR2_Tra_Diff = Algo_CR2_Tra_Diff + abs( True_CR2(1:3,4,i) - Algo_CR2(1:3,4,i) );
        
    end
    
    Tsai_CR1_Rot_Mean = Tsai_CR1_Rot_Diff / TRIALS;
    Tsai_CR2_Rot_Mean = Tsai_CR2_Rot_Diff / TRIALS;
    Algo_CR1_Rot_Mean = Algo_CR1_Rot_Diff / TRIALS;
    Algo_CR2_Rot_Mean = Algo_CR2_Rot_Diff / TRIALS;
    
    Tsai_CR1_Tra_Mean = Tsai_CR1_Tra_Diff / TRIALS;
    Tsai_CR2_Tra_Mean = Tsai_CR2_Tra_Diff / TRIALS;
    Algo_CR1_Tra_Mean = Algo_CR1_Tra_Diff / TRIALS;
    Algo_CR2_Tra_Mean = Algo_CR2_Tra_Diff / TRIALS;
    
    Tsai_CR1_Rot_Mean_x(num) = Tsai_CR1_Rot_Mean(1,1);
    Tsai_CR1_Rot_Mean_y(num) = Tsai_CR1_Rot_Mean(1,2);
    Tsai_CR1_Rot_Mean_z(num) = Tsai_CR1_Rot_Mean(1,3);
    Tsai_CR1_Rot_Mean_angle(num) = Tsai_CR1_Rot_Mean(1,4);
    Tsai_CR1_Tra_Mean_x(num) = Tsai_CR1_Tra_Mean(1,1);
    Tsai_CR1_Tra_Mean_y(num) = Tsai_CR1_Tra_Mean(2,1);
    Tsai_CR1_Tra_Mean_z(num) = Tsai_CR1_Tra_Mean(3,1);
    
    Algo_CR1_Rot_Mean_x(num) = Algo_CR1_Rot_Mean(1,1);
    Algo_CR1_Rot_Mean_y(num) = Algo_CR1_Rot_Mean(1,2);
    Algo_CR1_Rot_Mean_z(num) = Algo_CR1_Rot_Mean(1,3);
    Algo_CR1_Rot_Mean_angle(num) = Algo_CR1_Rot_Mean(1,4);
    Algo_CR1_Tra_Mean_x(num) = Algo_CR1_Tra_Mean(1,1);
    Algo_CR1_Tra_Mean_y(num) = Algo_CR1_Tra_Mean(2,1);
    Algo_CR1_Tra_Mean_z(num) = Algo_CR1_Tra_Mean(3,1);
    
    Tsai_CR2_Rot_Mean_x(num) = Tsai_CR2_Rot_Mean(1,1);
    Tsai_CR2_Rot_Mean_y(num) = Tsai_CR2_Rot_Mean(1,2);
    Tsai_CR2_Rot_Mean_z(num) = Tsai_CR2_Rot_Mean(1,3);
    Tsai_CR2_Rot_Mean_angle(num) = Tsai_CR2_Rot_Mean(1,4);
    Tsai_CR2_Tra_Mean_x(num) = Tsai_CR2_Tra_Mean(1,1);
    Tsai_CR2_Tra_Mean_y(num) = Tsai_CR2_Tra_Mean(2,1);
    Tsai_CR2_Tra_Mean_z(num) = Tsai_CR2_Tra_Mean(3,1);
    
    Algo_CR2_Rot_Mean_x(num) = Algo_CR2_Rot_Mean(1,1);
    Algo_CR2_Rot_Mean_y(num) = Algo_CR2_Rot_Mean(1,2);
    Algo_CR2_Rot_Mean_z(num) = Algo_CR2_Rot_Mean(1,3);
    Algo_CR2_Rot_Mean_angle(num) = Algo_CR2_Rot_Mean(1,4);
    Algo_CR2_Tra_Mean_x(num) = Algo_CR2_Tra_Mean(1,1);
    Algo_CR2_Tra_Mean_y(num) = Algo_CR2_Tra_Mean(2,1);
    Algo_CR2_Tra_Mean_z(num) = Algo_CR2_Tra_Mean(3,1);
end


figure(1)

annotation('textbox', [0 0.9 1 0.1], ...
    'String', '\fontsize{15} \bf Error Plots: Camera-Robot Calibration for R_1 Robot System with 10% Noise', ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center')

% Translation Vector Plots
num_frames = 10:10:100;
subplot(3,3,1)
plot(num_frames, Tsai_CR1_Tra_Mean_x, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR1_Tra_Mean_x, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{t}_X - \tilde{\bf{t}_X}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{t_X}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

subplot(3,3,2)
plot(num_frames, Tsai_CR1_Tra_Mean_y, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR1_Tra_Mean_y, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{t}_Y - \tilde{\bf{t}_Y}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{t_Y}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

subplot(3,3,3)
plot(num_frames, Tsai_CR1_Tra_Mean_z, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR1_Tra_Mean_z, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{t}_Z - \tilde{\bf{t}_Z}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{t_Z}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

% Rotation Vector Plots
subplot(3,3,4)
plot(num_frames, Tsai_CR1_Rot_Mean_x, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR1_Rot_Mean_x, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{R}_X - \tilde{\bf{R}_X}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{R_X}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

subplot(3,3,5)
plot(num_frames, Tsai_CR1_Rot_Mean_y, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR1_Rot_Mean_y, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{R}_Y - \tilde{\bf{R}_Y}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{R_Y}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

subplot(3,3,6)
plot(num_frames, Tsai_CR1_Rot_Mean_z, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR1_Rot_Mean_z, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{R}_Z - \tilde{\bf{R}_Z}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{R_Z}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

% Rotation Angle Plots
subplot(3,3,8)
plot(num_frames, Tsai_CR1_Rot_Mean_angle, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR1_Rot_Mean_angle, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{\theta} - \tilde{\bf{\theta}}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{\theta}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(2)

annotation('textbox', [0 0.9 1 0.1], ...
    'String', '\fontsize{15} \bf Error Plots: Camera-Robot Calibration for R_2 Robot System with 10% Noise', ...
    'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center')

% Translation Vector Plots
num_frames = 10:10:100;
subplot(3,3,1)
plot(num_frames, Tsai_CR2_Tra_Mean_x, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR2_Tra_Mean_x, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{t}_X - \tilde{\bf{t}_X}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{t_X}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

subplot(3,3,2)
plot(num_frames, Tsai_CR2_Tra_Mean_y, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR2_Tra_Mean_y, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{t}_Y - \tilde{\bf{t}_Y}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{t_Y}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

subplot(3,3,3)
plot(num_frames, Tsai_CR2_Tra_Mean_z, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR2_Tra_Mean_z, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{t}_Z - \tilde{\bf{t}_Z}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{t_Z}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

% Rotation Vector Plots
subplot(3,3,4)
plot(num_frames, Tsai_CR2_Rot_Mean_x, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR2_Rot_Mean_x, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{R}_X - \tilde{\bf{R}_X}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{R_X}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

subplot(3,3,5)
plot(num_frames, Tsai_CR2_Rot_Mean_y, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR2_Rot_Mean_y, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{R}_Y - \tilde{\bf{R}_Y}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{R_Y}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

subplot(3,3,6)
plot(num_frames, Tsai_CR2_Rot_Mean_z, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR2_Rot_Mean_z, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{R}_Z - \tilde{\bf{R}_Z}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{R_Z}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

% Rotation Angle Plots
subplot(3,3,8)
plot(num_frames, Tsai_CR2_Rot_Mean_angle, 'b','linewidth',3)
hold on
plot(num_frames, Algo_CR2_Rot_Mean_angle, 'r','linewidth',3)
legend('Tsai','Algo')
hold off
X_label = xlabel('\bf{Measurement Frames}')
set(X_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
Y_label = ylabel('$||\bf{\theta} - \tilde{\bf{\theta}}||$')
set(Y_label,'interpreter','latex','FontSize',16,'FontWeight','bold');
T_title = title('$\bf{\theta}$')
set(T_title,'interpreter','latex','FontSize',20,'FontWeight','bold');
set(gca,'fontsize',12)

cend = 0;































