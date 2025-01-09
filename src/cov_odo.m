function Q = cov_odo(Ut, k)
%COV_ODO Summary of this function goes here
%   Detailed explanation goes here
Q = eye(2);
% Distance parcourue
d = power(k*norm(Ut), 2); 
%Initialisation matrices
Q(1,1) = d;
Q(2,2)= d;
end

