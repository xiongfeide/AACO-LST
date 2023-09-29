function swarmLen= calSwarmLen(swarmPos,dists)

for i=1:length(swarmPos(1,1,:))
    swarmLen(i)=calLen(swarmPos(:,:,i),dists);
end

%% 计算某条线路的长度
function len=calLen(pos,dists)

len=0;
for i=1:length(pos(:,1))
    len=len+dists(pos(i,1),pos(i,2));
end
