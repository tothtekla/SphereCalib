function edgeIdxs = getIdxList(edgeImg)
%
% Save the idxs of the edges (color=255) from an N by M edge image
%
[N, M] = size(edgeImg);
edgeIdxs = zeros(N*M, 2);
numIdxs = 0;
for i = 1 : N
    for j = 1: M
        if(edgeImg(i, j) > 0)
            numIdxs = numIdxs + 1;
            edgeIdxs(numIdxs, :) = [i j];
        end
    end
end
edgeIdxs = edgeIdxs(1:numIdxs, :);
end