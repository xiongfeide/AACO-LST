%%=========================================================================
%  ACS.m
%%-------------------------------------------------------------------------
%%  主要符号说明
%%  C        n个城市的坐标，n×2的矩阵
%%  NC_max   最大迭代次数 100
%%  m        蚂蚁个数 50
%%  Alpha    表征信息素重要程度的参数-1
%%  Beta     表征启发式因子重要程度的参数4
%%  Rho      信息素蒸发系数0.1
%%  R_best   各代最佳路线
%%  L_best   各代最佳路线的长度
%%=========================================================================
function [Shortest_Route,L_best,L_ave]=ACS(tsplib,NC_max)
%% 第一步：变量初始化
m=50;
Alpha=2;
Beta=4;
Rho_E=0.3;  %信息素全局更新规则中，信息素挥发速率
Rho_L=0.1;  %信息素局部更新规则中，信息素挥发速率

q0=0.8;     %q0是一个伪随机比例规划中的参数.
loadpath=['E:\tsplib\',tsplib,'.tsp'];
[~,C]=readTSPFile(loadpath);%读取所需求解的TSP中包含的点的坐标值
n=size(C,1);        %表示问题的规模（城市个数n）得到矩阵的行数n为30
D=zeros(n,n);		%D表示完全图的赋权邻接矩阵 产生30*30矩阵，值全是0
for i=1:n
    for j=1:n
        if i~=j
            %D(i,j)= round(((C(i,1)-C(j,1))^2+(C(i,2)-C(j,2))^2)^0.5);
            D(i,j)= ((C(i,1)-C(j,1))^2+(C(i,2)-C(j,2))^2)^0.5;
        else
            D(i,j)=eps;
        end
    end
end
Eta=1./D;           %Eta为启发因子，这里设为距离的倒数
Lnn=NNM(D);
TauStart=(1/(n*Lnn)).*ones(n,n);       %TauStart为信息素有个初始值,10*10行矩阵
Tabu=zeros(m,n);         %存储并记录路径的生成,Tabu--->50*10矩阵
NC=1;                    %迭代计数器
R_best=zeros(NC_max,n);  %各代最佳路线100*10
L_best=inf.*ones(NC_max,1);   %各代最佳路线的长度100*1
L_ave=zeros(NC_max,1);        %各代路线的平均长度100*1
	Tau=TauStart;

while NC<=NC_max              %停止条件之一：达到最大迭代次数
%% 第二步：将m(50)只蚂蚁放到n(30)个城市上


  
    for i=1:m    
       Tabu(i,1)= unidrnd(n);   %unidrnd用于生成1..n的随机整数.设定m只蚂蚁出发的初始城市.
    end
    
    for i=1:m
        Tabu(i,1)= unidrnd(n);   %unidrnd用于生成1..n的随机整数.设定m只蚂蚁出发的初始城市.
    end   
    
   	 %% 第三步：m只蚂蚁按概率函数选择下一座城市，完成各自的周游
    for j=2:n
        for i=1:m
			%选择下一个访问的城市
            visited=Tabu(i,1:(j-1));	%已访问的城市---城市序号 visited也是矩阵
            J=zeros(1,(n-j+1));			%待访问的城市J,P都是一行值为0的矩阵
            P=J;						%待访问城市的选择概率分布
            Jc=1;
            for k=1:n
                if length(find(visited==k))==0
                    J(Jc)=k;
                    Jc=Jc+1;
                end
            end
			for k=1:length(J)
					 P(k)=(Tau(visited(end),J(k))^Alpha)*(Eta(visited(end),J(k))^Beta);
			end
			q=rand();
			if q<=q0       %采用使启发示信息与信息素量相互作用乘积最大的下一城市节点.
				Select=find(P==max(P));
            else           %否则,采用轮盘赌选择策略.
				P=P/(sum(P));
				Pcum=cumsum(P);
				Select=find(Pcum>=rand);
            end
            
            to_visit=J(Select(1));
            Tabu(i,j)=to_visit;
        end
		
		%信息素局部更新
        rw=Tabu(:,j-1);
        cm=Tabu(:,j);
		Tau(rw,cm)=(1-Rho_L).*Tau(rw,cm)+Rho_L.*TauStart(rw,cm);        %Tau(rw,cm)可以得到rw,cm的笛卡儿积
    end
	
    if NC>=2
        Tabu(1,:)=R_best(NC-1,:);%上一代最短路径作为本代第一条路径
    end
   
    %% 第四步：记录本次迭代最佳路线
    L=zeros(m,1);
    for i=1:m
        R=Tabu(i,:);
        for j=1:(n-1)
            L(i)=L(i)+D(R(j),R(j+1));
        end
        L(i)=L(i)+D(R(1),R(n));
    end
	L_best(NC)=min(L);
    pos=find(L==L_best(NC));
    R_best(NC,:)=Tabu(pos(1),:);
    L_ave(NC)=mean(L);
    NC=NC+1;
   
    %% 第五步：更新全局信息素（增加了全局最短路径的信息素）
    Delta_Tau=zeros(n,n);
	
    for j=1:(n-1)
        Delta_Tau(Tabu(pos(1),j),Tabu(pos(1),j+1))=Delta_Tau(Tabu(pos(1),j),Tabu(pos(1),j+1))+1/min(L);
%        Delta_Tau(Tabu(pos(1),j+1),Tabu(pos(1),j))=Delta_Tau(Tabu(pos(1),j),Tabu(pos(1),j+1));
    end
    Delta_Tau(Tabu(pos(1),n),Tabu(pos(1),1))=Delta_Tau(Tabu(pos(1),n),Tabu(pos(1),1))+1/min(L);
%     Delta_Tau(Tabu(pos(1),1),Tabu(pos(1),n))=Delta_Tau(Tabu(pos(1),n),Tabu(pos(1),1));
    
    rc=Tabu(pos(1),:);
    rw=rc;
    cm=[rc(2:end),rc(1)];
    
    Tau=(1-Rho_E).*Tau(rw,cm);
    Tau=(1-Rho_E).*Tau(cm,rw);
    Tau=Tau+Rho_E.*Delta_Tau;
%      Tau=(1-Rho_E)*Tau+Rho_E.*Delta_Tau;
    %% 第六步：禁忌表清零
   Tabu=zeros(m,n);
end

%% 第七步：输出结果
Pos=find(L_best==min(L_best));
Shortest_Route=R_best(Pos(1),:);
% Shortest_Length=L_best(Pos(1));
end