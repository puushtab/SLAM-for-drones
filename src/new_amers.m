function [Xt, Pt, A, B, Y_new] = new_amers(Xt, Pt, A, B, Y_new, k_y)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    N_1 = size(Xt);
    len_X = N_1(1);
    N_2 = size(Y_new);
    new_len = N_2(1);
    
    len = len_X + new_len;
    
    pos_to_add = ones(new_len, 1);
    pos_to_add(1:2:end) = Xt(1);
    pos_to_add(2:2:end) = Xt(2);

    Xt = [Xt; pos_to_add + Y_new];
    
    A = eye(len);
    B = [eye(2); zeros(len-2, 2)]; 

    % Agrandissement de Pt, nouvelles incertitudes données par Py /!\
    Py_new = cov_obs(Y_new, k_y);
    Pt = blkdiag(Pt, Py_new);
end