%% prepareTrajectory.m
% Script para preparar trajetória para uso no Simulink
% Carregar dados do cenário (exportados no Driving Scenario Designer)
load('pelpel.mat');  
pontos_referencia = data.ActorSpecifications.Waypoints;

%% Extrair coordenadas dos pontos de referência
x_pontos = pontos_referencia(:,1);              % coordenadas X
y_pontos = -pontos_referencia(:,2);             % coordenadas Y (invertido para ajustar convenção de eixo)

%% Calcular distâncias acumuladas ao longo do trajeto
matriz_distancia = squareform(pdist(pontos_referencia));  % matriz de distâncias entre pontos
comprimento_trechos = zeros(length(pontos_referencia)-1,1);
for i = 2:length(pontos_referencia)
    comprimento_trechos(i-1,1) = matriz_distancia(i,i-1); % distância entre ponto i e i-1
end
distancia_total = sum(comprimento_trechos); 
distancia_acumulada = cumsum([0; comprimento_trechos]);   % vetor de distâncias acumuladas
pontos_amostrados = linspace(0, distancia_total, 50);      % reamostragem em 50 pontos igualmente espaçados

%% Interpolar e suavizar trajetória
x_interpolado = interp1(distancia_acumulada, x_pontos, pontos_amostrados);
y_interpolado = interp1(distancia_acumulada, y_pontos, pontos_amostrados);
x_suavizado = smooth(pontos_amostrados, x_interpolado);    % trajetória X suavizada
y_suavizado = smooth(pontos_amostrados, y_interpolado);    % trajetória Y suavizada

%% Calcular orientação (ângulo de direção - theta/psi)
angulo_graus = zeros(length(pontos_amostrados),1);
for i = 2:length(pontos_amostrados)
    angulo_graus(i,1) = atan2d((y_interpolado(i)-y_interpolado(i-1)), ...
                               (x_interpolado(i)-x_interpolado(i-1)));
end
angulo_suavizado_graus = smooth(pontos_amostrados, angulo_graus);  % suavização do ângulo em graus

%% Calcular curvatura da trajetória
curvatura_trajetoria = calcularCurvatura(x_interpolado, y_interpolado);

%% ------------------- VARIÁVEIS EXIGIDAS PELO SIMULINK ------------------- %%

% 1. Variáveis para os blocos de Lookup Table (Curvatura e Referência XY)
gradbp = pontos_amostrados';         
curvature = curvatura_trajetoria';   

% Novas variáveis: Referências de X e Y mapeadas pela distância
xRef2s = x_suavizado'; 
yRef2s = y_suavizado'; 
Yrefs2 = y_suavizado'; % Criado também com essa grafia por precaução

% 2. Variáveis de Condição Inicial (Integradores de Parada e Posição)
X_o = x_suavizado(1);               
Y_o = y_suavizado(1);               
psi_o = deg2rad(angulo_suavizado_graus(1)); 
vel_o = 0; % Velocidade inicial 

% 3. Variável para tentar resolver o aviso do XY Plotter
velNorm = 1; 

% 4. Criar vetor de direção (1 = movimento para frente)
direcao_movimento = ones(length(pontos_amostrados),1);

disp('--- Condições iniciais extraídas para o Simulink ---');
fprintf('X_o: %.2f\n', X_o);
fprintf('Y_o: %.2f\n', Y_o);
fprintf('psi_o (rad): %.2f\n', psi_o);

%% Salvar e Forçar Carregamento no Workspace
save('refPath.mat', 'x_suavizado', 'y_suavizado', 'gradbp', 'curvature', ...
     'xRef2s', 'yRef2s', 'Yrefs2', ...
     'X_o', 'Y_o', 'psi_o', 'vel_o', 'velNorm', 'direcao_movimento');

% Força o carregamento no "Base Workspace"
load('refPath.mat');
disp('Variáveis xRef2s e yRef2s/Yrefs2 carregadas! O modelo deve conseguir ler as tabelas X/Y agora.');

%% Função auxiliar para cálculo da curvatura
function curvatura = calcularCurvatura(x, y)
    dX = gradient(x);
    d2X = gradient(dX);
    dY = gradient(y);
    d2Y = gradient(dY);
    curvatura = (dX .* d2Y - dY .* d2X) ./ (dX.^2 + dY.^2).^(3/2);
    curvatura(isnan(curvatura)) = 0;
    curvatura(isinf(curvatura)) = 0;
end