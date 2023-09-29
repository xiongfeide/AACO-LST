function swarm= initializeSwarm(numPoints,numSwarm,dists)
%% ��ʼ��swarm������ÿ��particle��position����particle��length�����������·�ĳ��ȣ�
for index=1:numSwarm
    swarmPos(:,:,index)=initializePos(numPoints);
end
swarmLen=calSwarmLen(swarmPos,dists);
swarm={swarmPos,swarmLen};
    %% ��ʼ������particle��position���Ӻ���������·��1-2-3-4-5-6��position��Ϊ[1 2],[2 3],[3 4],[4 5],[5 6],[6 1]
    function pos=initializePos(numPoints)       
        path=randperm(numPoints,numPoints);
        pos=devidePath(path);
        %% ��path�ָ��һ��һ�Ρ�
        %% ��ԭ��һ��pathΪ[1 2 3 4 5 6]������1�������ٷ���1��·��Ϊ1-2-3-4-5-6-1���ָ�Ϊ[1 2],[2 3],[3 4],[4 5],[5 6],[6 1]��
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