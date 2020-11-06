function [pv,pq] = fkeval(dq,q,dt)
vi=FKvelocity(dq,q,6);
[Jp,T]=calculateFK(q);
pold=Jp(6,:);
pv=pold+dt*vi(1:3)';
qnew=q+dt*dq;
[Jp,Tp]=calculateFK(qnew);
pq=Jp(6,:);
end
