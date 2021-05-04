function matC = markovActUpd3D(matA, matB, matC)
    %matC = zeros(size(matA,1), size(matA,2), size(matA,3));

    for m = 1:size(matC,1)
        for n = 1:size(matC,3)
            for i = 1:size(matC,2)
                for j = 1:size(matA,2)
                    for k = 1:size(matB,2)
                        if j - 1 + k == i
                            matC(m, i, n) = matC(m, i, n) + matA(m, j, n) * matB(m, k, n);
                        end
                    end
                end
            end
        end
    end
end