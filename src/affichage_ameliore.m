function affichage_ameliore(u_t, Xk, mat_cov, Xt)
    X_odom = u_t(1:2:end); 
    Y_odom = u_t(2:2:end);
    X_kalman = Xk(1:2:end);
    Y_kalman = Xk(2:2:end);
    
    %coordonnées des amers
    X_amers = Xt(3:2:end); 
    Y_amers = Xt(4:2:end);

    %incertitude
    len = size(Y_odom);
    sigma_drone = ones(len(1),1);
    sigma_drone = sigma_drone * sqrt(mat_cov(1,1));
    sigma_amers = sqrt(diag(mat_cov(3:2:end, 3:2:end)));

    figure;
    
    % Afficher la trajectoire prédite par l'odométrie
    subplot(1, 2, 1);
    plot(X_odom, Y_odom, '-o', 'DisplayName', 'Odométrie', 'Color', 'green', 'LineWidth', 2); 
    hold on;
    
    % Afficher son incertitude
    viscircles([X_odom, Y_odom], sigma_drone, 'EdgeColor', 'g', 'LineWidth', 1, 'LineStyle', '--');

    % Afficher les amers avec leur incertitude
    plot(X_amers, Y_amers, 's', 'DisplayName', 'Amers','Color', 'blue');
    viscircles([X_amers, Y_amers], sigma_amers, 'EdgeColor', 'b', 'LineWidth', 1, 'LineStyle', '--');

    % Afficher la trajectoire corrigée par le filtre de Kalman
    
    plot(X_kalman, Y_kalman, '-o', 'DisplayName', 'Filtre de Kalman', 'Color', 'red', 'LineWidth', 2);
    
    title('Comparaison entre la trajectoire predite et celle corrigée avec la position du drone et des amers avec incertitude');
    xlabel('Position X');
    ylabel('Position Y');
    legend('Location', 'Best');
    grid on;
    axis equal;
    
    % Afficher la matrice de covariance
    subplot(1, 2, 2);
    imagesc(mat_cov);
    title('Matrice de Covariance');
    colorbar;

    % Ajuster les propriétés de la figure
    sgtitle('Comparaison entre Odométrie et Filtre de Kalman avec Matrice de Covariance');
    

end

