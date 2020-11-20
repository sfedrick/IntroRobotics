function [Waypoints] = ItWerks(dims,Obstacles,q1,q2,plotStep,plot)
%RRT Summary of this function goes here

% initialize start/end node matrix and adjacency matrix
NodeStart=[q1];
EdgeStart=[0];
NodeEnd=[q2];
EdgeEnd=[0];

found=false;
colors=[1,1,0;
        1,1,0;
        1,1,0;
        1,1,0;
        1,1,0];
broken=false;
Age=0;
while(~found)
   
    counter = length(NodeStart);
    Age=Age+1;
     if(Age>10000)
        broken=true;
        break;
    end
    if(plot)
        obstacle(Obstacles,colors);
    end
    if(plot && mod(counter,plotStep)==0)        
        %plot the two trees 
        plotJointPos(NodeStart(end,:),'b',4);
        plotJointPos(NodeEnd(end,:),'r',4);
        axis equal;
        shg
    end
    
   % create a random point P in the config space
   P=randpoint(dims);
   startTree=length(NodeStart);
   endTree=length(NodeEnd);
   if(startTree>2*endTree)
       %sample near start tree
        ForceEndSearch=1;
     
   elseif(2*startTree<endTree)
       %sample near end tree
       ForceStartSearch=1;
   elseif(startTree==endTree || Age>1000)
       ForceStartSearch=0;
       ForceEndSearch=0;
   end
   
   if(ForceStartSearch)
       P=randPointmod(0.1,dims,NodeStart(end,:));
   end
   if(ForceEndSearch)
        P=randPointmod(0.1,dims,NodeEnd(end,:));
   end
   
   
   % calculate the index of closest node to random point P
   %startIdx is the closest node to P in start tree
   startIdx=closeNode(P,NodeStart);
   %endIdx is the closest node to P in end tree
   endIdx=closeNode(P,NodeEnd);
   
   % get the closest node and corresponding edge from start and end tree
   startNode=NodeStart(startIdx,:);
   endNode=NodeEnd(endIdx,:); 
   
   % for every point on the line (representing end effector position),
   % ensure that each link of robot has not collided w/ an obstacle
   Scheck1=checkLine(P,startNode, Obstacles);
   Echeck1=checkLine(P,endNode, Obstacles);
   
   % if Scheck1 is true and Echeck1 is false, add to start tree
   if (Scheck1 == true && Echeck1 == false)
       NodeStart = [NodeStart; P];
       % add edge to the edge matrix from startNode --> P
       EdgeStart = addEdge(EdgeStart,startIdx);
   % if Echeck1 is true and Scheck1 is false, add to end tree
   elseif (Echeck1 == true && Scheck1 == false)
       NodeEnd = [NodeEnd; P];
       % add edge to edge matrix from endNode --> P
       EdgeEnd = addEdge(EdgeEnd,endIdx);
   % if Scheck1 and Echeck1 are true, then we have connected the path
   elseif (Scheck1 == true && Echeck1 == true)
       found = true;
       NodeStart = [NodeStart; P];
       NodeEnd = [NodeEnd; P];
       EdgeStart = addEdge(EdgeStart,startIdx);
       EdgeEnd = addEdge(EdgeEnd,endIdx);
       % walk through matrix and record path
       Waypoints=ConnectTrees(EdgeStart,EdgeEnd,NodeStart,NodeEnd);
   end
   
end
if(broken)
    Waypoints=nan;
end
% walk through start and end trees to combine and get a single tree
%post porccess turn connect nodes into a list of waypoints
%post process

end

