function [routelen,path]=NNM(DistMatrix) %DistMatrix是邻接矩阵
    Ncities=length(DistMatrix);
    path=zeros(1,Ncities+1);
    path(1)=round(Ncities*rand+0.5);% p存储目前找到的所有城市的编号
    i=path(1);
    count=2;
    routelen=0;
	while count <= Ncities
     	NNdist= inf ;%NNdist存储目前找到的和当前城市距离最短的城市的距离
     	pp= i ;% i存储当前城市的编号 pp存储目前找到的城市编号
     	for j= 1: Ncities
          	if (DistMatrix(i, j) < NNdist) & (j~=i) & ((j~=path) == ones(1,length(path)))
                % 目标城市的要求为－－距离短、且不能是当前城市，也不能是以前已经走过的城市
                NNdist= DistMatrix(i, j) ; 
                pp= j ;
          	end           
        end
        routelen=routelen+NNdist;
     	path(count)=pp; 
        i= pp ;
     	count= count + 1 ;
    end
    path(count)=path(1);
    routelen=routelen+DistMatrix(path(count-1), path(count));
end