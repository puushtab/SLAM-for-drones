%Script principal
clear;
close all; %eviter les bugs
addpath (genpath('.')); %sous repertoire

% Spécifier le nom du fichier
nom_fichier = 'inputs/PartialObservation.data';

% Ouvrir le fichier en mode lecture
fid = fopen(nom_fichier, 'r');

% Lecture des premières perceptions
data = textscan(fid, "%s");
data = data{1};


% Initialiser variables i
k_y = 0.7;
k_odo = 0.02;
seuil = 2.5;

% Indice de lecture dans l'ensemble des données parsées
i_match = 3; % On commence à 3 pour aller chercher directement les perceps

Yt = [];
while ~strcmp(data{i_match}, 'odom')
    Yt = [Yt; str2double(data{i_match})];
    i_match = i_match+1;
end

[Xt, Pt, A, B, Ht] = init(Yt, k_y);

% Affichage carte
affichage(Xt,Pt);
pause(1);

% Initialisation variables d'affichage
u_t = zeros(2,1);
Xt_kalman = zeros(2,1);

s = size(data);
N_end = s(1);

% Lire le fichier ligne par ligne
while (i_match <= N_end) %TQ fichier non vide
    % Odometrie
    if(strcmp(data{i_match}, 'odom'))
        i_match = i_match + 2; % Skip odom et :

        % Stockage des données d'odométrie
        ut = zeros(2, 1);
        ut(1) = str2double(data{i_match});
        ut(2) = str2double(data{i_match+1});        
        
        i_match = i_match+2;

        Q = cov_odo(ut, k_odo);    

        % Prediction de l'état
        [Xt_star, Pt_star] = prediction_etat(Xt, ut, A, B, Pt, Q); %On a juste récup l'odométrie, RàS
        
        % Affichage de la carte
        affichage (Xt_star,Pt_star);
        pause(1);    
    end
    if(strcmp(data{i_match}, 'percep'))
        i_match = i_match + 2;
        Yt = [];
        while (i_match <= N_end && ~strcmp(data{i_match}, 'odom') )
            Yt = [Yt; str2double(data{i_match})];
            i_match = i_match+1;
        end

        [Ht, Yknown, Ynew] = obs_partiel(Xt, Yt, seuil);

        % Prediction sur la position
        [Xt_star, Pt_star] = prediction_etat(Xt, ut, A, B, Pt, Q);
            
        % Prediction sur l'observation
        [Yknown_star] = prediction_observateur(Xt_star, Ht);
    
        % Correction de la prediction
        P_Y = cov_obs(Yknown, k_y);
        [Xt, Pt] = correction_etat(Xt_star, Pt_star, Yknown_star, Yknown, Ht, P_Y);
        
        % Ajout des nouveaux amers
        [Xt, Pt, A, B] = new_amers(Xt, Pt, A, B, Ynew, k_y);
        
        % Affichage de la carte 
        affichage(Xt,Pt);
        pause(0.5);
    end
    u_t = [u_t; (u_t(end-1:end) + ut)]; %attention les 2 premières valeurs sont nulles
    Xt_kalman = [Xt_kalman; Xt(1:2)];
end

affichage_ameliore(u_t, Xt_kalman, Pt, Xt);

% Fermer le fichier
fclose(fid);

