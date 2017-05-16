function [r1_H_c1_Calculated, r2_H_c2_Calculated] = U2_CR_R1R2(c2t1_H_c1t1, r1t1_H_r1t2, r2t1_H_r2t2, c2t2_H_c1t2, ...
                                                               r1_H_c1_initialVal, r2_H_c2_initialVal)

    % Known calibrations obtained from the measurement data set                                                       
    A = c2t1_H_c1t1;
    B = r1t1_H_r1t2;
    C = r2t1_H_r2t2;
    D = c2t2_H_c1t2;
    
    % Initialization of the unknown X and Y
    init_X = r1_H_c1_initialVal;
    init_Y = r2_H_c2_initialVal;
    
    % Function to solve AX'BX = Y'CYD
    [X, Y] = AX_BX__Y_CYD_func_new(A, B, C, D, init_X, init_Y);
    
    r1_H_c1_Calculated = X;
    r2_H_c2_Calculated = Y;
end