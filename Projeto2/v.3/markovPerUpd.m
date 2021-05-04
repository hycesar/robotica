function matCNorm = markovPerUpd(matA, matB)
%MARKOV PERCEPTION UPDATE
%%%%%%%%%%%%%%%%%%%%%%%%
    matC = zeros(size(matA,1), size(matA,2));

    for i = 1:size(matA,2)
        matC(i) = matC(i) + matA(i) * matB(i);
    end
    matCNorm = matC/sum(matC);
end