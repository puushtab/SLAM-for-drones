function [Xtchap, Ptchap] = correction_etat(Xtstar, Ptstar, Ytstar, Yt, Ht, Py)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
Kt = Ptstar*Ht' / (Ht*Ptstar*Ht' + Py);
Xtchap = Xtstar + Kt * (Yt - Ytstar);
Ptchap = Ptstar - Kt * Ht * Ptstar;
end