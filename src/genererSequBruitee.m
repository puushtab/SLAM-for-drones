function genererSequBruitee (trajectoire_drone, positions_amers, fichier_sortie, r_det)
    % Paramètres du bruit
    sigma_distance = 0.1;  % Écart-type de la distance pour le bruit odométrique
    sigma_direction = 0.1; % Écart-type de la direction pour le bruit odométrique
    sigma_position_amers = 0.2;  % Écart-type de la position relative des amers
    
    % Calculer la distance totale parcourue
    distances = sqrt(sum(diff(trajectoire_drone).^2, 2));
    distance_totale = sum(distances);
    
    % Initialiser les matrices pour les observations et les odométries bruitées
    observations_bruitees = [];
    odometries_bruitees = [];

    % Boucler à travers la trajectoire pour ajouter du bruit
    for i = 2:size(trajectoire_drone, 1)
        % Calculer la distance et la direction entre les positions successives
        distance = distances(i-1);
        direction = atan2(trajectoire_drone(i, 2) - trajectoire_drone(i-1, 2), trajectoire_drone(i, 1) - trajectoire_drone(i-1, 1));
        % Ajouter du bruit aux observations
        observation_bruitee = distance + randn * sigma_distance;
        direction_bruitee = direction + randn * sigma_direction;
        
        % Générer une position relative bruitée pour chaque amers
        positions_relatives_bruitees = [];
        for j = 1:size(positions_amers, 1)
            position_relative = positions_amers(j, :) - trajectoire_drone(i, :);
            distance_amers_drone = norm(position_relative);

            % Mesurer uniquement les amers dans la plage de détection
            if distance_amers_drone <= r_det
                position_relative_bruitee = position_relative + randn(1, 2) * sigma_position_amers;
                positions_relatives_bruitees = [positions_relatives_bruitees; position_relative_bruitee];
            end
        end

        % Stocker les valeurs bruitées
        observations_bruitees = [observations_bruitees; observation_bruitee, direction_bruitee];
        odometries_bruitees = [odometries_bruitees; distance, direction];
    end

    % Écrire les données bruitées dans un fichier
    writematrix(observations_bruitees, [fichier_sortie '_observations.txt']);
    writematrix(odometries_bruitees, [fichier_sortie '_odométries.txt']);
    writematrix(positions_relatives_bruitees, [fichier_sortie '_positions_amers.txt']);


end

