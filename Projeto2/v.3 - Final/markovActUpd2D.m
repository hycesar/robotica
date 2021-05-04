function matC = markovActUpd2D(matA, matB)
    matC = zeros(size(matA,1), size(matA,2));

    for m = 1:size(matC,1)
        for i = 1:size(matC,2)
            for j = 1:size(matA,2)
                for k = 1:size(matB,2)
                    if j - 1 + k == i
                        matC(m, i) = matC(m, i) + matA(m, j) * matB(m, k);
                    end
                end
            end
        end
    end
end