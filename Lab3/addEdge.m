function [newEdge] = addEdge(Edge, nodeIdx)
% ADDEDGE adds to the adjacency matrix

[row,col] = size(Edge);

% add a column of length size(Edge) to Edge
newCol = zeros(row,1);
Edge = [Edge newCol];

% add a row of length size(Ege) to Edge
[row,col] = size(Edge);
newRow = zeros(1,col);
Edge = [Edge; newRow];

% set nodeIdx, last col to 1
Edge(nodeIdx, end) = 1;

% set last row, nodeIdx to -1
Edge(end, nodeIdx) = -1;

newEdge = Edge;

end

