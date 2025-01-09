%Script principal
clear;
close all;
addpath (genpath('.')); %sous repertoire

% Spécifier le nom du fichier
nom_fichier = 'inputs/FullObservation.data';

% Ouvrir le fichier en mode lecture
fid = fopen(nom_fichier, 'r');

% Lecture des premières perceptions
ligne = fgetl(fid);

format_percep = '%s : %f %f %f %f %f %f %f %f %f %f';
data_percep = textscan(ligne, format_percep, 'Delimiter', ' ');

% Initialiser variables
k_odo = 0.1;
k_y = 0.1;

Yt = zeros(10, 1);
for i = 1:10
    Yt(i) = data_percep{i+1};
end
[Xt, Pt, A, B, Ht] = init(Yt, k_y);

% Affichage carte
%affichage(Xt,Pt);
%pause(1);

% Initialisation variables d'affichage
u_t = zeros(2,1);
Xt_kalman = zeros(2,1);

% Lire le fichier ligne par ligne
while ~feof(fid) %TQ fichier non vide
    % Lecture de la ligne d'odométrie
    ligne = fgetl(fid);
    
    taille_ligne = size(ligne);
    if taille_ligne(1) == 1
        % Utiliser le format spécifié pour les données d'odométrie A CHANGER
        format_odom = '%s : %f %f';
        data_odom = textscan(ligne, format_odom, 'Delimiter', ' ');
    
        % Stockage des données d'odométrie
        donnees_odom = zeros(2, 1);
        for i = 1:2
            donnees_odom(i) = data_odom{i+1};
        end
        Q = cov_odo (donnees_odom, k_odo);
    
        % Prediction de l'état
        [Xt_star, Pt_star] = prediction_etat(Xt,donnees_odom, A, B,Pt,Q); %On a juste récup l'odométrie, RàS
        
        % Affichage de la carte
        %affichage (Xt_star,Pt_star);
        %pause(1);

        % Lecture de la ligne des perceptions
        ligne = fgetl(fid);
     
        % Utiliser le format spécifié pour les données de perception
        format_percep = '%s : %f %f %f %f %f %f %f %f %f %f';
        data_percep = textscan(ligne, format_percep, 'Delimiter', ' ');
        Yt = zeros(10, 1);
        for i = 1:10
            Yt(i) = data_percep{i+1};
        end
        P_Y = cov_obs(Yt, k_y);
    
        % Prediction sur la position
        [Xt_star,Pt_star] = prediction_etat(Xt,donnees_odom,A, B,Pt,Q);
    
        % Prediction sur l'observation
        [Yt_star] = prediction_observateur(Xt_star, Ht);
    
        % Correction de la prediction
        [Xt, Pt] = correction_etat(Xt_star, Pt_star, Yt_star, Yt, Ht, P_Y);
        
        % Affichage de la carte 
        %affichage(Xt,Pt);
        %pause(1);
    end
    u_t = [u_t; (u_t(end-1:end) + donnees_odom)]; %On suppose odométrie initialement nulle
    Xt_kalman = [Xt_kalman; Xt(1:2)];
end                
affichage_ameliore(u_t, Xt_kalman, Pt, Xt);

% Fermer le fichier
fclose(fid);

