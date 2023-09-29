function routelenAvg=getNNMavg(DistMatrix) %DistMatrix是邻接矩阵
   count=length(DistMatrix);
   routelenAvg=0;
   for i=1:count
       routelenAvg=routelenAvg+NNM2(i,DistMatrix);
   end
   routelenAvg=routelenAvg/count;
end