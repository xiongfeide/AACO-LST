function newSwarm=updateSwarm(swarm,swarmPBest,GBest,dists,omega,c1,c2)
%% 更新swarm，包括position，velocity，并且计算每条position的length。
swarmPos=swarm{1,1};
newSwarmVel=updateSwarmVel(swarm,swarmPBest,GBest,omega,c1,c2);
newSwarmPos=updateSwarmPos(swarmPos,newSwarmVel,dists);
newSwarmLen=calSwarmLen(newSwarmPos,dists);
newSwarm={newSwarmPos,newSwarmLen};

    %% 计算swarm中所有particle的velocity
    function newSwarmVel=updateSwarmVel(swarm,swarmPBest,GBest,omega,c1,c2)
        
        swarmPos1=swarm{1,1};
        numSwarm=length(swarmPos1(1,1,:));
        swarmPBestPos=swarmPBest{1,1};
        GBestPos=GBest{1,1};
        
        for i1=1:numSwarm
            newSwarmVel{1,i1}=updateVel(swarmPos1(:,:,i1),swarmPBestPos(:,:,i1),GBestPos,omega,c1,c2);
        end
        %% 计算swarm中某particle的velocity
        function vel=updateVel(pos,PBestPos,GBestPos,omega,c1,c2)
            numPos=length(pos(:,1));
            v1=calIniVel(numPos);
            v1=[v1(:,1:2),v1(:,3)*omega];
            v2=posMinusPos(c1,pos,PBestPos);
            v3=posMinusPos(c2,pos,GBestPos);
            vel=velPlusVel(numPos,v1,v2);
            vel=velPlusVel(numPos,vel,v3);
            
            %% 初始化单个particle的velocity
            function vel=calIniVel(numPos)
                index=1;
                vel=[];
                while index<=numPos
                    temp=randperm(numPos,2);
                    if isempty(vel)
                        vel(1,:)=temp;
                        index=index+1;
                    else
                        tempVelocity=vel(vel(:,1)==temp(1,1),:);
                        tempVelocity=tempVelocity(temp(1,2)==tempVelocity(:,2),:);
                        if isempty(tempVelocity)
                            vel(index,:)=temp;
                            index=index+1;
                        end
                    end
                end
                possibilities=rand(numPos,1);
                vel(:,3)=possibilities;
            end
            
            %% position-position操作，生成新的velocity
            function vel=posMinusPos(c,pos1,pos2)
                pos1=getSymPos(pos1);
                pos2=getSymPos(pos2);
                vel=examine(pos1,pos2);
                possibilities=rand(length(vel(:,1)),1)*c;
                possibilities(possibilities>1)=1;
                vel(:,3)=possibilities;
                
                %% 得到对称的pos
                function pos=getSymPos(pos)
                    symPos=[pos(:,2) pos(:,1)];
                    pos=[pos;symPos];
                    pos=sortVelOrPos(pos);
                end
                %% 检验pos2中是否存在pos1中没有的段的子函数，即为pos2-pos1操作
                function pos2=examine(pos1,pos2)
                    index=1;
                    while index<=length(pos2(:,1))
                        tempPos1=pos1(pos2(index,1)==pos1(:,1),:);
                        tempPos11=tempPos1(pos2(index,2)==tempPos1(:,2),:);
                        if ~isempty(tempPos11)
                            pos2(index,:)=[];
                        else
                            index=index+1;
                        end
                    end
                end
            end
            %% velocity+velocity操作
            function vel=velPlusVel(numPoints,vel1,vel2)
                vel=[];
                for i2=1:numPoints
                    tempVel1=vel1(i2==vel1(:,1),:);
                    tempVel2=vel2(i2==vel2(:,1),:);
                    if ~isempty(tempVel1)
                        if ~isempty(tempVel2)
                            index1=1;
                            index2=1;
                            temp=[];
                            while index1<=length(tempVel1(:,1))
                                flag=0;
                                while index2<=length(tempVel2(:,1))
                                    if tempVel1(index1,2)==tempVel2(index2,2)
                                        if tempVel1(index1,3)>tempVel2(index2,3)
                                            temp=tempVel1(index1,:);
                                        else
                                            temp=tempVel2(index2,:);
                                        end
                                        tempVel1(index1,:)=[];
                                        tempVel2(index2,:)=[];
                                        flag=1;
                                        break;
                                    else
                                        index2=index2+1;
                                    end
                                end
                                if flag~=1
                                    index1=index1+1;
                                end
                                index2=1;
                            end
                            vel=[vel;tempVel1];
                            vel=[vel;tempVel2];
                            vel=[vel;temp];
                        else
                            vel=[vel;tempVel1];
                        end
                    else
                        if ~isempty(tempVel2)
                            vel=[vel;tempVel2];
                        end
                    end
                end
                vel=avoidInconsistency(vel); % avoiding inconsistency of the velocities of different dimensions
                %% 对于symmetry TSP，velocity可能存在inconsistency of the velocities of different dimensions的问题，解决如下
                function vel=avoidInconsistency(vel)
                    symVel=vel(:,[2 1 3]);
                    index3=1;
                    index4=1;
                    while index3<=length(vel(:,1))
                        flag1=0;
                        while index4<=length(symVel(:,1))
                            if vel(index3,1)==symVel(index4,1) && vel(index3,2)==symVel(index4,2)
                                if vel(index3,3)>=symVel(index4,3)
                                    symVel(index4,:)=[];
                                    break;
                                else
                                    vel(index3,:)=[];
                                    flag1=1;
                                    break;
                                end
                            else
                                index4=index4+1;
                            end
                        end
                        index4=1;
                        if flag1==0
                            index3=index3+1;
                        end
                    end
                    vel=[vel;symVel];
                    vel=sortVelOrPos(vel);
                end
            end
        end
    end
    %% 更新swarm的pos，即产生新的路径
    function newSwarmPos=updateSwarmPos(swarmPos,newSwarmVel,dists)
        
        numSwarm=length(swarmPos(1,1,:));
        for i3=1:numSwarm
            newSwarmPos(:,:,i3)=updatePos(swarmPos(:,:,i3),newSwarmVel{1,i3},dists);
        end
        %% 更新pos，即产生一个新的路径
        function newPos=updatePos(pos,vel,dists)
            newPos=[];
            numPos=length(pos(:,1));
            symPos=[pos(:,2),pos(:,1)];
            pos=[pos;symPos];
            
            alpha=rand(1);
            cut=vel(vel(:,3)>=alpha,:);
            if ~isempty(cut)
                [~,tempIndex]=max(cut(:,3));
                e=cut(tempIndex,1:2);
            else
                e=getShortestArc(pos,dists);
            end
            newPos=[newPos;e];
            
            while length(newPos(:,1))<numPos
                newPosLen=length(newPos(:,1));
                currentDim=newPos(newPosLen,2);
                if length(newPos(:,1))~=numPos-1
                    cut=eliminate(newPos,currentDim,cut);
                    tempCut=cut(currentDim==cut(:,1),:);
                    if ~isempty(tempCut)
                        [~,tempIndex]=max(tempCut(:,3));
                        e=tempCut(tempIndex,1:2);
                    else
                        pos=eliminate(newPos,currentDim,pos);
                        tempPos=pos(currentDim==pos(:,1),:);
                        if ~isempty(tempPos)
                            e=getShortestArc(tempPos,dists);
                        else
                            temp=linspace(1,numPos,numPos);
                            temp=temp(temp~=currentDim)';
                            E=[ones(numPos-1,1)*currentDim,temp];
                            E=eliminate(newPos,currentDim,E);
                            e=getShortestArc(E,dists);
                        end
                    end
                    newPos=[newPos;e];
                else
                    newPos=[newPos;[currentDim,newPos(1,1)]];
                end
            end
            %% 用于消除cut或者pos中已经被选过的边的子函数
            function cut=eliminate(newPos,currentDim,cut)
                newPos(:,2)=[];
                newPos=[newPos,ones(length(newPos(:,1)),1)*currentDim];
                for i4=1:length(newPos(:,1))
                    I1=find(cut(:,1)==newPos(i4,1));
                    I2=cut(I1,2)==newPos(i4,2);
                    index5=I1(I2);
                    I3=find(cut(:,1)==newPos(i4,2));
                    I4=cut(I3,2)==newPos(i4,1);
                    index6=I3(I4);
                    cut([index5 index6],:)=[];
                end
            end
            %% 筛选出最短的边的子函数
            function e=getShortestArc(pos,dists)
                tempDists=[];
                for i5=1:length(pos(:,1))
                    tempDists=[tempDists;dists(pos(i5,1),pos(i5,2))];
                end
                [~,index7]=min(tempDists);
                e=pos(index7,:);
            end
        end
    end
    function result=sortVelOrPos(beforeSort)
        [~,I]=sort(beforeSort(:,1));
        result=beforeSort(I,:);
    end
end
