
%% Function to solve AX'BX = Y'CYD
function [X, Y] = AX_BX__Y_CYD_function(A, B, C, D, init_X, init_Y)

    % Determining the number of data sets
    M = size(A, 3);
    
    % Initialized Rotation Matrices using which true rotation matrices will
    % be determined
    init_Rx = init_X(1:3, 1:3, 1);
    init_Ry = init_Y(1:3, 1:3, 1);
    
    % Iterative method for estimation of Rotational part
    for iteration_count = 1:30
        
        % Loop for collecting all measurement data sets
        for i = 1:M
    
            % Obtaining respective rotation matrices from the received data
            % sets
            Ra = A(1:3, 1:3, i);
            Rb = B(1:3, 1:3, i);
            Rc = C(1:3, 1:3, i);
            Rd = D(1:3, 1:3, i);
            
            % Formulation of: F * del_r = q 
            % values to be used in calculation of q(:,:,i)
            q_row = -Ra * init_Rx' * Rb * init_Rx + init_Ry' * Rc * init_Ry * Rd;
            
            % values to be used in calculation of F(:,:,i)
            RaRx_Rb = Ra * init_Rx' * Rb;
            Rx_RbRx = init_Rx' * Rb * init_Rx; 
            
            Ry_Rc = init_Ry' * Rc;
            RyRd = init_Ry * Rd;
            Ry_RcRyRd = init_Ry' * Rc * init_Ry * Rd;

            % Calculation of F for i'th data set 
            F(:,:,i) = [ -RaRx_Rb * screwMatrix(init_Rx(:,1)) + Ra * screwMatrix(Rx_RbRx(:,1)),   Ry_Rc * screwMatrix(RyRd(:,1)) - screwMatrix(Ry_RcRyRd(:,1));
                         -RaRx_Rb * screwMatrix(init_Rx(:,2)) + Ra * screwMatrix(Rx_RbRx(:,2)),   Ry_Rc * screwMatrix(RyRd(:,2)) - screwMatrix(Ry_RcRyRd(:,2));
               %          -RaRx_Rb * screwMatrix(init_Rx(:,3)) + Ra * screwMatrix(Rx_RbRx(:,3)),   Ry_Rc * screwMatrix(RyRd(:,3)) - screwMatrix(Ry_RcRyRd(:,3))
                       ];

            % Calculation of q for i'th data set
            q(:,:,i) =  [   q_row(:,1);
                            q_row(:,2);
               %             q_row(:,3);
                        ];
        end

        % 3D to 2D array conversion for F (F is 9x9m matrix after conversion)
        midVal = permute(F,[1 3 2]);
        F_2D = reshape(midVal,[],size(F,2),1);

        % 3D to 2D array conversion for q (q is 1x9m matrix after conversion)
        midVal = permute(q,[1 3 2]);
        q_2D = reshape(midVal,[],size(q,2),1);

        % Pseudo-Inverse calculation
        [U,S,V] = svd(F_2D);
        r = rank(F_2D);
        Finv = (V(:,1:r) / S(1:r,1:r)) * U(:,1:r)';
        
        % Estimated rotation vector
        del_r = Finv * q_2D;
        
        % Breaking down the estimated rotation vector into respective
        % rotation vectors of the unknown X and Y
        del_rx = del_r(1:3);
        del_ry = del_r(4:6);

        % New initial value of Roatations for next iteration using the
        % respective rotation vectors
        init_Rx = expm(screwMatrix(del_rx)) * init_Rx;
        init_Ry = expm(screwMatrix(del_ry)) * init_Ry;
    end
    
    % Final estimated rotation matrices for X and Y after the competion of
    % iteration loop
    Rx_calculated = init_Rx;
    Ry_calculated = init_Ry;
    
    % Difference between true and estimated values of rotation 
    %     disp('error = ');
    %     disp(true_X(1:3,1:3) - Rx_calculated);
    %     disp(true_Y(1:3,1:3) - Ry_calculated);
    
    % Estimation of Translational part using results from Rotational part
    for i = 1:M
        
        % Obtaining respective rotation matrices from the received data
        % sets
        Ra = A(1:3, 1:3, i);
        Rb = B(1:3, 1:3, i);
        Rc = C(1:3, 1:3, i);
        
        % Obtaining respective translation vectors from the received data
        % sets
        ta = A(1:3,4,i);
        tb = B(1:3,4,i);
        tc = C(1:3,4,i);
        td = D(1:3,4,i);
        
        % Formulation of: J * t = p 
        % Calculation of J for i'th data set 
        J(:,:,i) = [Ra * Rx_calculated' * Rb - Ra * Rx_calculated',     -Ry_calculated' * Rc + Ry_calculated'];
        
        % Calculation of p for i'th data set 
        p(:,:,i) = -ta - Ra * Rx_calculated' * tb + Ry_calculated' * tc + Ry_calculated' * Rc * Ry_calculated * td;
    end
    
    % 3D to 2D array conversion for J (J is 9x9m matrix after conversion)
    midVal = permute(J,[1 3 2]);
    J_2D = reshape(midVal,[],size(J,2),1);

    % 3D to 2D array conversion for p (p is 1x9m matrix after conversion)
    midVal = permute(p,[1 3 2]);
    p_2D = reshape(midVal,[],size(p,2),1);
        
    % Pseudo-Inverse calculation
    [U,S,V] = svd(J_2D);
    r = rank(J_2D);
    %     Jinv = V(:,1:r)*inv(S(1:r,1:r))*U(:,1:r)';
    Jinv = (V(:,1:r) / S(1:r,1:r)) * U(:,1:r)';
    
    % Estimated translation vector
    t_calculated = Jinv * p_2D;
    
    % Breaking down the estimated translation vector into respective
    % translation vectors of the unknown X and Y
    tx_calculated = t_calculated(1:3);
    ty_calculated = t_calculated(4:6);
    
    
    % Combining estimated Roatational and Translational parts
    X = [Rx_calculated, tx_calculated; 0, 0, 0, 1];
    Y = [Ry_calculated, ty_calculated; 0, 0, 0, 1];

end




