function affichage(X,P)
%coordonnées du drone
X_drone = X(1);
Y_drone = X(2);
%coordonnées des amers
X_amers = X(3:2:end-1); %-1 ?
Y_amers = X(4:2:end);
%incertitude
sigma_drone = sqrt(P(1,1));
sigma_amers = sqrt(diag(P(3:2:end, 3:2:end)));

% Créer une nouvelle figure
figure;

% Afficher le drone avec son incertitude
scatter(X_drone, Y_drone, 'filled', 'Marker', 'o', 'DisplayName', 'Drone');
hold on;
viscircles([X_drone, Y_drone], sigma_drone, 'EdgeColor', 'b', 'LineWidth', 1, 'LineStyle', '--');

% Afficher les amers avec leur incertitude
scatter(X_amers, Y_amers, 'filled', 'Marker', 'o', 'DisplayName', 'Amers');
viscircles([X_amers, Y_amers], sigma_amers, 'EdgeColor', 'r', 'LineWidth', 1, 'LineStyle', '--');

% Configurer le graphique
title('Position du drone et des amers avec incertitude');
xlabel('Position X');
ylabel('Position Y');
legend('Location', 'Best');
grid on;
axis equal;
hold off;
end

