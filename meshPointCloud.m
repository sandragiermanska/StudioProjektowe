%point cloud
pc = pcread("data/framesTest3Limited/frameLimited1.ply");

%% ver1

%meshing

mesh(pc.Location);

%% ver2

%pc = mergedPointClouds1{1, 1};
k = boundary(double(pc.Location(:,1)), double(pc.Location(:,2)), double(pc.Location(:,3)));
figure;
trisurf(k,pc.Location(:,1),pc.Location(:,2),pc.Location(:,3),'Facecolor','red','FaceAlpha',0.1)

%% ver3

gridStep = 0.1;
pc = pcdownsample(pc, 'gridAverage', gridStep);
% delete floor
roi = [pc.XLimits(1)-0.1 pc.XLimits(2)
    pc.YLimits(1) pc.YLimits(2)
    pc.ZLimits(1) pc.ZLimits(2)];
indices = findPointsInROI(pc, roi);
pc = select(pc, indices);
k = boundary(double(pc.Location(:,1)), double(pc.Location(:,2)), double(pc.Location(:,3)));
figure;
trisurf(k,pc.Location(:,1),pc.Location(:,2),pc.Location(:,3),'Facecolor','red','FaceAlpha',0.1)
