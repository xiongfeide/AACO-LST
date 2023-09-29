function plotGBest(GBest,points,iteration)

GBestPos=GBest{1,1};
GBestLen=GBest{1,2};
path=[GBestPos(:,1);GBestPos(1,1)]';
points=points(path,:);
plot(points(:,1),points(:,2),'o-')
points=roundn(points,-1);
for i=1:length(points(:,1))
    text(points(i,1),points(i,2),['(',num2str(points(i,1)),',',num2str(points(i,2)),')'])
end
xlabel('����');
ylabel('γ��');
title(['���ȣ�',num2str(GBestLen),' ','����������',num2str(iteration)]);

