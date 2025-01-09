function [X_0, P_0, A, B, H_0] = init(y, k_y)
%prend en entrée un premier vecteur d’observations 𝑦 et 
% qui renvoie le vecteur d état 𝑋0chap contenant
% les positions initiales du drone 
% (supposée égale à [0 0]𝑇 et des amers, la matrice de covariance P0chap
% et les matrices 𝐴, 𝐵 et 𝐻0 , tous initialisés.)

n = size(y);
len = n(1); %Taille vecteur mesure, 2 de moins que vecteur X
X_0 = zeros(len+2,1); %Position initiale drones et amers
X_0(3:(len+2)) = y;

A = eye(len+2); %Id, précisé dans le texte
B = [eye(2); zeros(len, 2)]; %Transforme ut (taille 2) en vecteur avec des 0 ensuite

H_0 = [zeros(len, 2), eye(len)]; %Matrice qui fait le calcul de X à Y
H_0(1:2:end, 1) = -1;
H_0(2:2:end, 2) = -1;

Y_0 = H_0*X_0; 

Py = cov_obs(Y_0, k_y);
P_0 = eye(len+2);

% Confiance accordée à la valeur d'origine à 0.1
P_0(1,1) = 0.1; 
P_0(2,2) = 0.1;
for i = 3:(len+2)
    P_0(i,i) = Py(i-2,i-2);
end

end

