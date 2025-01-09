function M = cov_obs(Y_t, k)
%COV Summary of this function goes here
%Yt est juste le vecteur observation, 
%Prendre incertitude large pour x, y
N = size(Y_t);
len = N(1);
M = eye(len);
for i = 1:2:len
%Incertitude issue de la distance    
    M(i,i) = power(k*norm(Y_t(i:(i+1))), 2); 
    M(i+1, i+1) = power(k*norm(Y_t(i:(i+1))), 2);
end
end

