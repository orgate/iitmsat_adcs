function TS=Targetstates(omega,TargetQuat,num)
TS=zeros(7,num-1);
    for t=1:num-1
        TS(:,t)=cat(1,omega(:,t),TargetQuat(:,t));
    end