function [Nodes,Edges] = RRT(dims,Obstacles,q1,q2)
%RRT Summary of this function goes here

% initialize start/end node matrix and adjacency matrix
NodeStart=[q1];
EdgeStart=[];
NodeEnd=[q2];
EdgeEnd=[];

found=false;

while(~found)
    
   % compute a random point P in the 3D space
   P=randpoint(dims);
   
   % calculate the index of closest node to random point P
   startIdx=closeNode(P,NodeStart);
   endIdx=closeNode(P,NodeEnd);
   
   % get the closest node and corresponding edge from start and end tree
   startNode=NodeStart(startIdx);
   endNode=NodeEnd(endIdx); 
   
   % check if every discretized point on the line is within joint limits 
   % and also that each link is not collided w/ an obstacle
   Scheck1=checkLine(P,startNode);
   Echeck1=checkLine(P,endNode);
   
   % if Scheck1 is true and Echeck1 is false, add to start tree
   if (Scheck1 == true && Echeck1 == false)
       NodeStart = [NodeStart; P];
       % add edge to the edge matrix from startNode --> P
   end
   % if Echeck1 is true and Scheck1 is false, add to end tree
   if (Echeck1 == true && Scheck1 == false)
       NodeEnd = [NodeEnd; P];
       % add edge to edge matrix from endNode --> P
   end
   % if Scheck1 and Echeck1 are true, then we have connected the path
   if (Scheck1 == true && Echeck1 == true)
       found = true;
       % walk through matrix and record path
       
   end
end

% walk through start and end trees to combine and get a single tree

end

