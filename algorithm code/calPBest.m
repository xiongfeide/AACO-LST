function swarmPBest=calPBest(swarmPBest,swarm)
if isempty(swarmPBest)
    swarmPBest=swarm;
else
    PBestPos=swarmPBest{1,1};
    PBestLen=swarmPBest{1,2};
    swarmPos=swarm{1,1};
    swarmLen=swarm{1,2};
    flag=swarmLen<PBestLen;
    PBestPos(:,:,flag)=swarmPos(:,:,flag);
    PBestLen(:,flag)=swarmLen(:,flag);
    swarmPBest={PBestPos,PBestLen};
end