function [GBest,L_best]=testPSO(tsplib,numIteration)
% numPoints=70;%求解的TSP包含的点的个数
numSwarm=40;%粒子群中粒子的数量
omega=0.8;%omega、c1、c2速度更新时所需参数
c1=2;
c2=2;

loadpath=['E:\tsplib\',tsplib,'.tsp'];
[~,points]=readTSPFile(loadpath);%读取所需求解的TSP中包含的点的坐标值
numPoints=length(points);
points=points(1:numPoints,:);

dists=calDists(points);%计算各点之间的距离，得到一个矩阵
swarm=initializeSwarm(numPoints,numSwarm,dists);%粒子群的初始化，计算各个粒子的位置
swarmPBest=calPBest([],swarm);%记录每个粒子的最优解
GBest=calGBest([],swarm);%记录整个种群得到过的最优解

L_best=zeros(numIteration,1);%记录每次迭代的GBest中最优路径长度
for i=1:numIteration
    swarm=updateSwarm(swarm,swarmPBest,GBest,dists,omega,c1,c2);%更新粒子群，即更新粒子的速度和位置
    swarmPBest=calPBest(swarmPBest,swarm);%记录每个粒子的最优解
    GBest=calGBest(GBest,swarm);%记录整个种群得到过的最优解
    L_best(i)=GBest{1,2};
%     drawnow%drawnow可以将每次迭代的图形绘制出来
%     plotGBest(GBest,points,i);%画图，将最优解的路线绘制出来
end