function GBest=calGBest(GBest,swarm)

swarmPos=swarm{1,1};
swarmLen=swarm{1,2};
[~,I]=min(swarmLen);
swarmBestPos=swarmPos(:,:,I);
swarmBestLen=swarmLen(:,I);
if isempty(GBest)
    GBest={swarmBestPos,swarmBestLen};
else
    GBestLen=GBest{1,2};
    if swarmBestLen<GBestLen
        GBest={swarmBestPos,swarmBestLen};
    end
end
