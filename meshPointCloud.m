%point cloud
pc = pcread("data/framesTest3Limited/frameLimited1.ply");
pc = pcdownsample(pc, 'gridAverage', 0.000001);

%% final version

x1 = double(pc.Location(:, 1));
y1 = double(pc.Location(:, 2));
z1 = double(pc.Location(:, 3));
x = double(x1);
y = double(y1);
z = double(z1);

tri = delaunay(x,y);
tr = triangulation(tri, x, y, z);
trisurf(tr);

%% former version
x1 = double(pc.Location(:, 1));
y1 = double(pc.Location(:, 2));
z1 = double(pc.Location(:, 3));
P = [x1 y1 z1];
P = alphaShape(P, 0.055);
figure;
plot(P);

%% ver1
mesh(pc.Location);

%% ver2
pc = pcdownsample(pc, 'gridAverage', 0.000001);

k = boundary(double(pc.Location(:,1)), double(pc.Location(:,2)), double(pc.Location(:,3)));
figure;
trisurf(k,pc.Location(:,1),pc.Location(:,2),pc.Location(:,3),'Facecolor','red','FaceAlpha',0.1)

%% ver3

gridStep = 0.00001;
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


