function [rMin, rMax] = findR (angleStep, paramStep)
 anglephi=0:(2*pi)/angleStep:2*pi;
 angletheta=0:(2*pi)/angleStep:2*pi;
  rMin = ones(length(anglephi))*inf;
  rMax = zeros(length(anglephi));
  
   jlim1=[0,2*pi];
  jlim2=[0,2*pi];
  jlim3=[-1.4,1.7];
  jlim4=[-1.9,1.7];
  jlim5=[-2,1.5];
%   jlim1=[-1.4,1.4];
%   jlim2=[-1.2,1.4];
%   jlim3=[-1.4,1.7];
%   jlim4=[-1.9,1.7];
%   jlim5=[-2,1.5];
  
 
    % step through all possible positions of each joint
    for joint1 = jlim1(1):(abs(jlim1(1))+abs(jlim1(2)))/paramStep:jlim1(2)
        for joint2 = jlim2(1):(abs(jlim2(1))+abs(jlim2(2)))/paramStep:jlim2(2)
            for joint3 = jlim3(1):(abs(jlim3(1))+abs(jlim3(2)))/paramStep:jlim3(2)
                for joint4 = jlim4(1):(abs(jlim4(1))+abs(jlim4(2)))/paramStep:jlim4(2)
                    for joint5 = jlim5(1):(abs(jlim5(1))+abs(jlim5(2)))/paramStep:jlim5(2)
                        d=[joint1,joint2,joint3,joint4,joint5];
                        % output XYZ coord of that joint change this with
                        % test function
                        [x,y,z]=testgetxyz(d); 
                        %[x,y,z]=getxyz(d);                        
                        
                        
                        % convert XYZ coord into spherical coords r
                        [r,theta,phi]=Cartesian2Spherical(x,y,z);
                        
                        % search through rMin for phi and theta of coord and check if rMin
                        % at (phi, theta) =< r
                        [rminny,minnycol,minnyrow]=searchR(rMin,theta,phi,angleStep);
                        if(r<rminny)
                            rMin(minnyrow,minnycol)=r;
                            
                            if((minnycol-1)>0)
                                rMin(minnyrow,minnycol-1)=r;
                            end
                            if((minnyrow-1)>0)
                                rMin(minnyrow-1,minnycol)=r;
                            end
                            if((minnycol+1)<length(anglephi))
                                rMin(minnyrow,minnycol+1)=r;
                            end
                            if((minnyrow+1)<length(anglephi))
                                rMin(minnyrow+1,minnycol)=r;
                            end
                            if((minnyrow-1)>0 && (minnycol-1)>0)
                                rMin(minnyrow-1,minnycol-1)=r;
                            end
                            if( (minnyrow+1)<length(anglephi)&& (minnycol+1)<length(anglephi))
                                rMin(minnyrow+1,minnycol+1)=r;
                            end
                            if((minnyrow+1)<length(anglephi) && (minnycol-1)>0)
                                rMin(minnyrow+1,minnycol-1)=r;
                            end
                            
                             if((minnyrow-1)>0 && (minnycol+1)<length(anglephi))
                                rMin(minnyrow-1,minnycol+1)=r;
                            end
                            
                        end

                        % search through rMax for phi and theta of coord and check if rMax
                        % at (phi, theta) >= r
                        [rmaxxy,maxxycol,maxxyrow]=searchR(rMax,theta,phi,angleStep);
                        if(r>=rmaxxy)
                            rMax(maxxyrow,maxxycol)=r;
                            
                            
                            if((maxxycol-1)>0)
                                rMax(maxxyrow,maxxycol-1)=r;
                            end
                            if((maxxyrow-1)>0)
                                rMax(maxxyrow-1,maxxycol)=r;
                            end
                            if((maxxycol+1)<length(anglephi))
                                rMax(maxxyrow,maxxycol+1)=r;
                            end
                            if((maxxyrow+1)<length(anglephi))
                                rMax(maxxyrow+1,maxxycol)=r;
                            end
                            if((maxxyrow-1)>0 && (maxxycol-1)>0)
                                rMax(maxxyrow-1,maxxycol-1)=r;
                            end
                            if( (maxxyrow+1)<length(anglephi)&& (maxxycol+1)<length(anglephi))
                                rMax(maxxyrow+1,maxxycol+1)=r;
                            end
                            if((maxxyrow+1)<length(anglephi) && (maxxycol-1)>0)
                                rMax(maxxyrow+1,maxxycol-1)=r;
                            end
                            
                             if((maxxyrow-1)>0 && (maxxycol+1)<length(anglephi))
                                rMax(maxxyrow-1,maxxycol+1)=r;
                            end
                            
                            
                        end
                        
                        
                         
                    end
                end
            end
        end
    end
               
 
    
end