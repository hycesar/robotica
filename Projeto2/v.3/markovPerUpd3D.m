function matCNorm = markovPerUpd3D(matA, matB, matC)
%MARKOV PERCEPTION UPDATE
%%%%%%%%%%%%%%%%%%%%%%%%
    %matC = zeros(size(matA,1), size(matA,2), size(matA,3));

    for m = 1:size(matA,1)
        for n = 1:size(matA,3)
            for i = 1:size(matA,2)
                matC(m, i, n) = matC(m, i, n) + matA(m, i, n) * matB(m, i, n);
            end
        end
    end
    if sum(sum(sum(matC))) == 0
        matCNorm = matC;
    else
        matCNorm = matC/sum(sum(sum(matC)));
    end
end