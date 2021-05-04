function matCNorm = markovPerUpd2D(matA, matB)
%MARKOV PERCEPTION UPDATE
%%%%%%%%%%%%%%%%%%%%%%%%
    matC = zeros(size(matA,1), size(matA,2));

    for m = 1:size(matA,1)
        for i = 1:size(matA,2)
            matC(m, i) = matC(m, i) + matA(m, i) * matB(m, i);
        end
    end
    matCNorm = matC/sum(sum(matC));
end