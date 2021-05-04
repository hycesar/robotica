function matC = markovActUpd(matA, matB)
%MARKOV ACTION UPDATE
%%%%%%%%%%%%%%%%%%%%%%%%
    matC = zeros(size(matA,1), size(matA,2));

    for i = 1:size(matC,2)
        for j = 1:size(matA,2)
            for k = 1:size(matB,2)
                if j - 1 + k == i
                    matC(i) = matC(i) + matA(j) * matB(k);
                end
            end
        end
    end
end