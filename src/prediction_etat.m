function [Xtstar, Ptstar] = prediction_etat(Xchap, ut, A, B, Pchap, Q)
Xtstar = A*Xchap + B*ut;
Ptstar = A*Pchap*A' + B*Q*B';
end