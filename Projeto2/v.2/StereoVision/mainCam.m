%%
% PASSO B: remover codigo antigo, preferencialmente desconecte as cameras
%   clear all no terminal;

%
% As cameras são sensíveis, não se desconectam facilmente.
%
%%
% PASSO O: Conectar as câmeras e ver seu estado.
[cam1, cam2] = connectStereoCams;

%
% Recebe os parâmetros da Câmera que serão utilizados em função que irá
% retornar a distância da câmera para a da cor-objetivo na tela.
%

%%
% PASSO P: Calibrar a câmera que pode estar em posição alterada.
%stereoParams = quickStereoCalibration(cam1, cam2);

%
% Recebe os parâmetros da Câmera que serão utilizados em função que irá
% retornar a distância da câmera para a da cor-objetivo na tela.
%

%%
% PASSO N: Use getHSVColor(path) para estimar a média do valor de HSV.
%LAB = getLABColor(cam1, 81);

%
% Recebe uma imagem ao qual o usuário fará a seleção manual da cor, por
% meio de cliques no monitor, de modo a detectar cor e faixa de tolerância.
% Recebe a área em pixels² a ser considerada reduzindo ruído e imprecisão
% do clique. No fim, haverá uma matriz com M linhas (quantidade de cores a
% serem detectadas) e N colunas (cada cor detectada em cada clique).
%

vetorPos = zeros(1, 3);

while 1
    %%
    % PASSO M: Use a média de cada cor para detecção na imagem passada e passe
    % para função colorDetectHSV(imagem, corLab)
    stat1 = detectLABColor(cam1, LAB);
    debug('b', vetorPos);       %apenas debug.REMOVER
    stat2 = detectLABColor(cam2, LAB);
    debug('w', vetorPos);       %apenas debug.REMOVER
    %
    % Ao fim, será repassada a posição relativa de cada cor em uma matriz, onde
    % linha M é cada uma das posições da cor, em ordem de área, e cada coluna N
    % é uma posição no plano cartesiano X, Y, Z, no quarto quadrante, apenas Z
    % positivo.
    %

    %%
    % PASSO K: Verificar consistência e só depois disso, triangular.

    vetorPos = triangular(stat1, stat2, stereoParams);

    %
    % Faz algumas checagens para ver se tá tudo ok, para então retornar uma
    % posição X, Y e Z válida.
    %

    %%
    % PASSO V: Guardar em arquivo que será enviado a Arduíno

end


