%{
Caracterización del sistema BÁSICA
Usa métodos numéricos para identificar los parámetros del sistema
desde la respuesta al escalón experimental
%}

function caracterizacion_basica()
    close all; clc;
    
    % ========== LEER DATOS EXPERIMENTALES ==========
    data = readmatrix('Modelo_físico.csv', 'NumHeaderLines', 1, 'Delimiter', ';', 'DecimalSeparator', ',');
    
    tiempo = data(:, 1);    % segundos
    distancia = data(:, 2); % centímetros
    
    fprintf('=== CARACTERIZACIÓN DEL SISTEMA (Método Básico) ===\n');
    fprintf('Datos cargados: %d puntos\n', length(tiempo));
    fprintf('Tiempo: %.2f s - %.2f s\n', tiempo(1), tiempo(end));
    fprintf('Distancia: %.2f cm - %.2f cm\n\n', distancia(1), distancia(end));
    
    % ========== RESPUESTA AL IMPULSO ==========
    % h(t) = d/dt[respuesta al escalón]
    dt = diff(tiempo);
    dy = diff(distancia);
    impulso = dy ./ dt;
    tiempo_impulso = tiempo(1:end-1) + dt/2;
    
    fprintf('Respuesta al impulso: %d puntos\n\n', length(impulso));
    
    % ========== ANÁLISIS DE LA RESPUESTA AL ESCALÓN ==========
    % Valor final (ganancia DC)
    K_dc = distancia(end);
    fprintf('Ganancia DC (K): %.4f cm\n', K_dc);
    
    % Normalizar la respuesta (0 a 1)
    distancia_norm = distancia / K_dc;
    
    % Método 1: Sistema de primer orden - Ajuste de curva exponencial
    % Modelo: y(t) = K * (1 - exp(-t/tau))
    
    % Encontrar el tiempo donde alcanza el 63.2% (1-1/e) del valor final
    idx_63 = find(distancia_norm >= 0.632, 1, 'first');
    if ~isempty(idx_63)
        tau = tiempo(idx_63);
        fprintf('\nConstante de tiempo (tau): %.4f s\n', tau);
        fprintf('Polo del sistema de 1er orden: s = %.4f\n', -1/tau);
    else
        tau = tiempo(end) / 3; % Estimación
        fprintf('\nConstante de tiempo estimada: %.4f s\n', tau);
    end
    
    % Modelo de primer orden: G(s) = K / (tau*s + 1)
    sys_1orden = tf(K_dc, [tau, 1]);
    fprintf('\nFunción de transferencia (1er orden):\n');
    fprintf('G(s) = %.4f / (%.4f*s + 1)\n', K_dc, tau);
    
    % Método 2: Sistema de segundo orden subamortiguado
    % y(t) = K * (1 - exp(-zeta*wn*t) * (cos(wd*t) + (zeta*wn/wd)*sin(wd*t)))
    
    % Buscar sobrepico
    [max_val, idx_max] = max(distancia);
    overshoot_percent = ((max_val - distancia(end)) / distancia(end)) * 100;
    
    fprintf('\n=== ANÁLISIS DE 2DO ORDEN ===\n');
    
    if overshoot_percent > 0.5 % Si hay sobrepico significativo
        fprintf('Sobrepico detectado: %.2f%%\n', overshoot_percent);
        
        % Calcular zeta desde el sobrepico
        % Mp = exp(-pi*zeta/sqrt(1-zeta^2))
        Mp = overshoot_percent / 100;
        zeta = -log(Mp) / sqrt(pi^2 + log(Mp)^2);
        
        % Tiempo del pico
        t_peak = tiempo(idx_max);
        
        % Frecuencia natural amortiguada
        wd = pi / t_peak;
        
        % Frecuencia natural
        wn = wd / sqrt(1 - zeta^2);
        
        fprintf('Factor de amortiguamiento (zeta): %.4f\n', zeta);
        fprintf('Frecuencia natural (wn): %.4f rad/s\n', wn);
        fprintf('Frecuencia amortiguada (wd): %.4f rad/s\n', wd);
        
        % Función de transferencia de segundo orden
        num = K_dc * wn^2;
        den = [1, 2*zeta*wn, wn^2];
        sys_2orden = tf(num, den);
        
    else
        fprintf('No se detectó sobrepico significativo\n');
        fprintf('El sistema parece sobreamortiguado o críticamente amortiguado\n');
        
        % Estimar como sistema críticamente amortiguado
        % Ts ≈ 4.75*tau para sistema de 2do orden
        t_settling = tiempo(find(abs(distancia_norm - 1) < 0.02, 1, 'first'));
        if isempty(t_settling)
            t_settling = tiempo(end);
        end
        
        zeta = 1; % Críticamente amortiguado
        wn = 4.75 / t_settling;
        
        fprintf('Asumiendo amortiguamiento crítico (zeta = 1)\n');
        fprintf('Frecuencia natural estimada (wn): %.4f rad/s\n', wn);
        
        num = K_dc * wn^2;
        den = [1, 2*zeta*wn, wn^2];
        sys_2orden = tf(num, den);
    end
    
    fprintf('\nFunción de transferencia (2do orden):\n');
    fprintf('G(s) = %.4f / (s^2 + %.4f*s + %.4f)\n', num, den(2), den(3));
    
    % Polos del sistema
    polos = roots(den);
    fprintf('\nPolos del sistema:\n');
    fprintf('  p1 = %.4f%+.4fi\n', real(polos(1)), imag(polos(1)));
    fprintf('  p2 = %.4f%+.4fi\n', real(polos(2)), imag(polos(2)));
    
    % ========== VALIDACIÓN ==========
    fprintf('\n=== VALIDACIÓN DE MODELOS ===\n');
    
    % Simular ambos modelos
    t_sim = linspace(0, tiempo(end), 200);
    [y1, ~] = step(sys_1orden, t_sim);
    [y2, ~] = step(sys_2orden, t_sim);
    
    % Calcular error RMS
    y1_interp = interp1(t_sim, y1, tiempo, 'linear', 'extrap');
    y2_interp = interp1(t_sim, y2, tiempo, 'linear', 'extrap');
    
    rmse_1 = sqrt(mean((distancia - y1_interp).^2));
    rmse_2 = sqrt(mean((distancia - y2_interp).^2));
    
    fprintf('RMSE (1er orden): %.4f cm\n', rmse_1);
    fprintf('RMSE (2do orden): %.4f cm\n', rmse_2);
    
    % R-squared
    ss_tot = sum((distancia - mean(distancia)).^2);
    ss_res_1 = sum((distancia - y1_interp).^2);
    ss_res_2 = sum((distancia - y2_interp).^2);
    
    r2_1 = 1 - (ss_res_1 / ss_tot);
    r2_2 = 1 - (ss_res_2 / ss_tot);
    
    fprintf('R² (1er orden): %.4f (%.2f%%)\n', r2_1, r2_1*100);
    fprintf('R² (2do orden): %.4f (%.2f%%)\n', r2_2, r2_2*100);
    
    % ========== GRÁFICAS ==========
    figure('Position', [100, 100, 1400, 600], 'Color', 'w');
    
    % Subplot 1: Comparación de modelos con datos
    subplot(2, 2, 1);
    hold on;
    plot(tiempo, distancia, 'ko-', 'LineWidth', 2.5, 'MarkerSize', 8, ...
         'DisplayName', 'Datos Experimentales', 'MarkerFaceColor', 'k');
    plot(t_sim, y1, 'b--', 'LineWidth', 2, 'DisplayName', ...
         sprintf('Modelo 1er Orden (R²=%.3f)', r2_1));
    plot(t_sim, y2, 'r-', 'LineWidth', 2, 'DisplayName', ...
         sprintf('Modelo 2do Orden (R²=%.3f)', r2_2));
    grid on;
    xlabel('Tiempo (s)', 'FontSize', 11);
    ylabel('Distancia (cm)', 'FontSize', 11);
    title('Respuesta al Escalón: Datos vs Modelos', 'FontSize', 13, 'FontWeight', 'bold');
    legend('Location', 'southeast', 'FontSize', 9);
    hold off;
    
    % Subplot 2: Respuesta al impulso
    subplot(2, 2, 2);
    plot(tiempo_impulso, impulso, 'mo-', 'LineWidth', 2, 'MarkerSize', 6, ...
         'MarkerFaceColor', 'm');
    grid on;
    xlabel('Tiempo (s)', 'FontSize', 11);
    ylabel('Velocidad (cm/s)', 'FontSize', 11);
    title('Respuesta al Impulso (d/dt de Escalón)', 'FontSize', 13, 'FontWeight', 'bold');
    
    % Subplot 3: Diagrama de polos (1er orden)
    subplot(2, 2, 3);
    pzmap(sys_1orden);
    grid on;
    title('Polos y Ceros - Modelo 1er Orden', 'FontSize', 13, 'FontWeight', 'bold');
    
    % Subplot 4: Diagrama de polos (2do orden)
    subplot(2, 2, 4);
    pzmap(sys_2orden);
    grid on;
    title('Polos y Ceros - Modelo 2do Orden', 'FontSize', 13, 'FontWeight', 'bold');
    
    % ========== FIGURA 2: RESPUESTA AL IMPULSO COMPARADA ==========
    figure('Position', [150, 150, 1000, 500], 'Color', 'w');
    
    % Impulso de los modelos identificados
    [h1, t_h1] = impulse(sys_1orden, tiempo(end));
    [h2, t_h2] = impulse(sys_2orden, tiempo(end));
    
    hold on;
    plot(tiempo_impulso, impulso, 'ko-', 'LineWidth', 2, 'MarkerSize', 7, ...
         'DisplayName', 'Impulso Experimental (derivada)', 'MarkerFaceColor', 'k');
    plot(t_h1, h1, 'b--', 'LineWidth', 2.5, 'DisplayName', 'Impulso Modelo 1er Orden');
    plot(t_h2, h2, 'r-', 'LineWidth', 2.5, 'DisplayName', 'Impulso Modelo 2do Orden');
    grid on;
    xlabel('Tiempo (s)', 'FontSize', 12);
    ylabel('Amplitud', 'FontSize', 12);
    title('Comparación de Respuestas al Impulso', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 11);
    hold off;
    
    % ========== EXPORTAR RESULTADOS ==========
    fprintf('\n=== GUARDANDO RESULTADOS ===\n');
    
    % Guardar workspace
    save('modelos_caracterizados.mat', 'sys_1orden', 'sys_2orden', ...
         'tiempo', 'distancia', 'impulso', 'tiempo_impulso', ...
         'K_dc', 'tau', 'zeta', 'wn', 'r2_1', 'r2_2', 'rmse_1', 'rmse_2');
    fprintf('Datos guardados en: modelos_caracterizados.mat\n');
    
    % Archivo de texto con resultados
    fid = fopen('resultados_caracterizacion.txt', 'w');
    fprintf(fid, '=== CARACTERIZACIÓN DEL SISTEMA ===\n');
    fprintf(fid, 'Fecha: %s\n\n', datestr(now));
    fprintf(fid, 'DATOS EXPERIMENTALES:\n');
    fprintf(fid, '  Puntos: %d\n', length(tiempo));
    fprintf(fid, '  Tiempo total: %.4f s\n', tiempo(end));
    fprintf(fid, '  Valor final: %.4f cm\n\n', distancia(end));
    fprintf(fid, 'MODELO DE PRIMER ORDEN:\n');
    fprintf(fid, '  G(s) = %.4f / (%.4f*s + 1)\n', K_dc, tau);
    fprintf(fid, '  Constante de tiempo: %.4f s\n', tau);
    fprintf(fid, '  Polo: %.4f\n', -1/tau);
    fprintf(fid, '  RMSE: %.4f cm\n', rmse_1);
    fprintf(fid, '  R²: %.4f (%.2f%%)\n\n', r2_1, r2_1*100);
    fprintf(fid, 'MODELO DE SEGUNDO ORDEN:\n');
    fprintf(fid, '  G(s) = %.4f / (s² + %.4f*s + %.4f)\n', num, den(2), den(3));
    fprintf(fid, '  Factor de amortiguamiento: %.4f\n', zeta);
    fprintf(fid, '  Frecuencia natural: %.4f rad/s\n', wn);
    fprintf(fid, '  Polos: %.4f %+.4fi, %.4f %+.4fi\n', ...
            real(polos(1)), imag(polos(1)), real(polos(2)), imag(polos(2)));
    fprintf(fid, '  RMSE: %.4f cm\n', rmse_2);
    fprintf(fid, '  R²: %.4f (%.2f%%)\n', r2_2, r2_2*100);
    fclose(fid);
    fprintf('Resultados guardados en: resultados_caracterizacion.txt\n');
    
    fprintf('\n¡Caracterización completada exitosamente!\n');
    fprintf('El modelo de %s orden tiene mejor ajuste (R²=%.4f)\n', ...
            iif(r2_2 > r2_1, '2do', '1er'), max(r2_1, r2_2));
end

function result = iif(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
