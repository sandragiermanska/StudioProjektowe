%% Section 1 - chosen frames

%numbers of chosen frames
numbersOfFrame = [4 6 16 71 21 26 29 32 39 47 52 59 56 63];
sizeNumbers = size(numbersOfFrame, 2);

%viewing chosen frames
figure;
for i = 1 : sizeNumbers
   subplot(2, ceil(sizeNumbers / 2), i);
   pcshow(pcread("data/framesTest3Limited/frameLimited" + num2str(numbersOfFrame(i)) + ".ply"), 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
end

%% Section 2 - merging 

%fixed point cloud
fixedPC = pcread("data/framesTest3Limited/frameLimited" + num2str(numbersOfFrame(1)) + ".ply");

%results of merges
mergedPointClouds = {};

%iterating across point clouds
for i = 2 : sizeNumbers

    %fixed point cloud
    figure;
    subplot(2, 4, 1);
    pcshow(fixedPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("fixedPC");
    
    %translating closer to center
    M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -fixedPC.ZLimits(2) 1];
    tform = affine3d(M);
    fixedPC = pctransform(fixedPC, tform); 
    subplot(2, 4, 2);
    pcshow(fixedPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("fixedPC centered");
    
    %removing noises
    fixedPC = pcdenoise(fixedPC);
    
    %moving point cloud
    movingPC = pcread("data/framesTest3Limited/frameLimited" + num2str(numbersOfFrame(i)) + ".ply"); 
    subplot(2, 4, 5);
    pcshow(movingPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("movingPC");

    %translating closer to center
    M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -movingPC.ZLimits(2) 1];
    tform = affine3d(M);
    movingPC = pctransform(movingPC, tform);
    subplot(2, 4, 6);
    pcshow(movingPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("movingPC centered");

    %removing noises
    movingPC = pcdenoise(movingPC);

    %getting downsamples
    fixedPCDownSample = pcdownsample(fixedPC, 'gridAverage', 0.0001);
    movingPCDownSample = pcdownsample(movingPC, 'gridAverage', 0.0001);

    %rigid trasformation
    tform = pcregrigid(movingPCDownSample, fixedPCDownSample, 'Verbose', true, 'Metric', 'pointToPlane', 'Extrapolate', true);

    %tranforming
    pointCloudTransform = pctransform(movingPC, tform);
    subplot(2, 4, [3 7]);
    pcshow(pointCloudTransform, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("pointCloudTransform");

    %merging pointclouds
    mergeSize = 0.0001;
    pointCloudMerge = pcmerge(fixedPC, pointCloudTransform, mergeSize);
    subplot(2, 4, [4 8]);
    pcshow(pointCloudMerge, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("pointCloudMerge");
    
    %adding result of merge to cell array
    mergedPointClouds(1, size(mergedPointClouds) + 1) = {pointCloudMerge};
    
    %setting new fixed point cloud to moving one
    fixedPC = movingPC;
end

%% Section 3 - first phase of merge

%viewing results
figure;
for i = 1 : size(mergedPointClouds, 2)
   subplot(2, ceil(size(mergedPointClouds, 2) / 2), i);
   pcshow(mergedPointClouds{1, i}, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
end

%% Section 4 - merging merges

%fixed point cloud
fixedPC = mergedPointClouds{1, 1};

%results of merges
mergedPointClouds1 = mergedPointClouds;
mergedPointClouds2 = {};

while (size(mergedPointClouds1, 2) > 2)

    %iterating across point clouds
    for i = 1 : 2 : size(mergedPointClouds1, 2) - 2

        %fixed point cloud
        fixedPC = mergedPointClouds1{1, i};
        figure;
        subplot(2, 4, 1);
        pcshow(fixedPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
        title("fixedPC");

        %translating closer to center
        M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -fixedPC.ZLimits(2) 1];
        tform = affine3d(M);
        fixedPC = pctransform(fixedPC, tform); 
        subplot(2, 4, 2);
        pcshow(fixedPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
        title("fixedPC centered");

        %removing noises
        fixedPC = pcdenoise(fixedPC);

        %moving point cloud
        movingPC = mergedPointClouds{1, i + 1};
        subplot(2, 4, 5);
        pcshow(movingPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
        title("movingPC");

        %translating closer to center
        M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -movingPC.ZLimits(2) 1];
        tform = affine3d(M);
        movingPC = pctransform(movingPC, tform);
        subplot(2, 4, 6);
        pcshow(movingPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
        title("movingPC centered");

        %removing noises
        movingPC = pcdenoise(movingPC);

        %getting downsamples
        fixedPCDownSample = pcdownsample(fixedPC, 'gridAverage', 0.0001);
        movingPCDownSample = pcdownsample(movingPC, 'gridAverage', 0.0001);

        %rigid trasformation
        tform = pcregrigid(movingPCDownSample, fixedPCDownSample, 'Verbose', true, 'Metric', 'pointToPlane', 'Extrapolate', true);

        %tranforming
        pointCloudTransform = pctransform(movingPC, tform);
        subplot(2, 4, [3 7]);
        pcshow(pointCloudTransform, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
        title("pointCloudTransform");

        %merging pointclouds
        mergeSize = 0.0001;
        pointCloudMerge = pcmerge(fixedPC, pointCloudTransform, mergeSize);
        subplot(2, 4, [4 8]);
        pcshow(pointCloudMerge, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
        title("pointCloudMerge");

        %adding result of merge to cell array
        mergedPointClouds2(1, size(mergedPointClouds2, 2) + 1) = {pointCloudMerge};

    end

    mergedPointClouds1 = mergedPointClouds2;
    mergedPointClouds2 = {};
end

%% Section 5 - last before merge

%viewing results
figure;
for i = 1 : size(mergedPointClouds1, 2)
   subplot(1, 2, i);
   pcshow(mergedPointClouds1{1, i}, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
end

%% Section 6 - merging two last frames

%point clouds
pc1 = mergedPointClouds1{1, 1};
pc2 = mergedPointClouds1{1, 2};

%rotation
thetaY = pi - pi / 6;
rot = [cos(thetaY) 0 sin(thetaY) 0;
       0 1 0 0;
       -sin(thetaY) 0 cos(thetaY) 0;
       0 0 0 1];
tform = affine3d(rot);
pc2 = pctransform(pc2, tform);

%viewing results of rotation
figure;
subplot(1, 2, 1);
pcshow(pc1, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
subplot(1, 2, 2);
pcshow(pc2, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');

%transformation 
[tr, pc] = pcregrigid(pc2, pc1, 'Verbose', true, 'Extrapolate', true, 'Metric', 'PointToPlane');
transform = pctransform(pc2, tr);
figure;
pcshow(transform, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');

%viewing results of rotation
figure;
pcshow(pc1, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
hold on;
pcshow(transform, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
hold off;

%move to merge
M = [1 0 0 0; 
    0 1 0 0; 
    0 0 1 0; 
    (pc1.XLimits(2) - transform.XLimits(2)) (-pc1.YLimits(1) + transform.YLimits(1)) (-pc1.ZLimits(2) + transform.ZLimits(2)) 1]; %-transform.ZLimits(2)
tform = affine3d(M); 
pc1 = pctransform(pc1, tform); 

%final merge
pcMerge = pcmerge(pc1, transform, 0.0001);
figure;
pcshow(pcMerge, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
xlabel("X");
ylabel("Y");
zlabel("Z");
