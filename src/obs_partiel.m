function [Ht, Y_known, Y_new] = obs_partiel(Xt_chap, Yt, seuil)
% Fonction qui va calculer la distance entre chaque amer et perception
% et qui va associer à chaque amer une perception par distance
% croissante, en priorisant les amers perçus et en "ignorant" les cas
% problématiques de couplage en équidistance. On ignore les distances au
% delà du seuil de perception.
    % Dimensions de la matrice
    s = size(Yt);
    N_perceptions = s(1)/2;
    s = size(Xt_chap);
    N_amers = (s(1)-2)/2;

    X_amers = Xt_chap(3:end);

    % Création d'un vecteur des positions absolues des perceptions
    % Création d'un vecteur avec la position du drone répétée à ajouter
    pos_to_add = ones(2*N_perceptions, 1);
    pos_to_add(1:2:end) = Xt_chap(1);
    pos_to_add(2:2:end) = Xt_chap(2);

    Yt_abs = Yt + pos_to_add;

    dist_matrix = zeros(N_amers, N_perceptions);
    for i = 1:N_amers
        for j = 1:N_perceptions
            dist_matrix(i,j) = norm(X_amers(2*i-1:2*i)- Yt_abs(2*j-1:2*j));
        end
    end
    % On va:
    %   - Créer un tableau d'association entre les perceptions et amers
    %   - Initialiser minimum
    %   - TQ minimum < seuil (distance qu'on peut tirer)
    %       - Trouver minimum et indices associés
    %       - Vérifier qu'il soit en dessous du seuil
    %       - Rajouter l'associations dans le tableau
    %       - Remplacer les distances par +INF pour pas retirer indices
    associations = ones(1, N_perceptions)*(-1);
    
    N_known = 0; % Donne la dimension de Yknown
    minimum = min(dist_matrix(:)); % On récupère min pour extraire
    amer_min = 0;
    perc_min = 0;
    while (minimum < seuil) % Tant qu'il y a des distances < seuil 
        for i = 1:N_amers
            for j = 1:N_perceptions
                if dist_matrix(i,j) == minimum % On choisit la connexion
                    amer_min = i;
                    perc_min = j;
                end
            end
        end
        associations(perc_min) = amer_min; % On enregistre la connexion

        dist_matrix(amer_min, :) = Inf; % On rend inutile toutes les autres
        dist_matrix(:, perc_min) = Inf;
        
        minimum = min(dist_matrix(:));% On considère la connexion minimale

        N_known = N_known + 1;
    end

    Y_known = zeros(2*N_known, 1);
    Y_new = zeros(2*(N_perceptions-N_known), 1);

    Ht = zeros(2*N_known, 2*N_amers+2);

    count_k = 1;
    count_n = 1;
    for j = 1:N_perceptions
        i = associations(j); % On récupère "l'indice" de l'amer associé
        if i ~= (-1) % Si il existe une association
            % Diagonales de (-1) pour soustraire la position
            Ht(count_k, 1) = (-1);%
            Ht(count_k+1, 2) = (-1);
            % Diagonale de 1 pour ajouter perception
            Ht(count_k, 2 + (2*i-1)) = 1;
            Ht(count_k+1, 2 + (2*i)) = 1; 
            % Ajout des valeurs à Y_known
            Y_known(count_k) = Yt(2*j-1);
            Y_known(count_k+1) = Yt(2*j);
            % Itère à travers les lignes de Ht et Y_known
            count_k = count_k+2; 
        else
            % Ajout des valeurs à Y_new
            Y_new(count_n) = Yt(2*j-1);
            Y_new(count_n+1) = Yt(2*j);
            % Itère les lignes de Y_new
            count_n = count_n + 2;
        end    
    end
end