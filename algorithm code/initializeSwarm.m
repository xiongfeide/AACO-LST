function swarm= initializeSwarm(numPoints,numSwarm,dists)
%% 初始化swarm，包括每个particle的position，和particle的length（即代表的线路的长度）
for index=1:numSwarm
    swarmPos(:,:,index)=initializePos(numPoints);
end
swarmLen=calSwarmLen(swarmPos,dists);
swarm={swarmPos,swarmLen};
    %% 初始化单个particle的position的子函数，例如路线1-2-3-4-5-6，position即为[1 2],[2 3],[3 4],[4 5],[5 6],[6 1]
    function pos=initializePos(numPoints)       
        path=randperm(numPoints,numPoints);
        pos=devidePath(path);
        %% 将path分割成一段一段。
        %% 如原先一段path为[1 2 3 4 5 6]，即由1出发，再返回1，路径为1-2-3-4-5-6-1。分割为[1 2],[2 3],[3 4],[4 5],[5 6],[6 1]。
        function pos=devidePath(path)
            for i=1:length(path(1,:))
                if i~=length(path(1,:))
                    pos(i,1)=path(1,i);
                    pos(i,2)=path(1,i+1);
                else
                    pos(i,1)=path(1,i);
                    pos(i,2)=path(1,1);
                end
            end
        end
    end
end