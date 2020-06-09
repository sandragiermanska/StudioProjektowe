%% merging from one side
%% Section 1 - chosen frames

%numbers of chosen frames
numbersOfFrame = [4 6 16 71 21 26 29 32 39 47 52 59 56 63]; 
sizeNumbers = size(numbersOfFrame, 2);


%viewing chosen frames
%{
figure;
for i = 1 : sizeNumbers
   subplot(2, ceil(sizeNumbers / 2), i);
   pcshow(pcread("data/framesTest3Limited/frameLimited" + num2str(numbersOfFrame(i)) + ".ply"), 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
end
%}

%% Section 2 - merging 

%fixed point cloud
fixedPC = pcread("data/framesTest3Limited/frameLimited" + num2str(numbersOfFrame(1)) + ".ply");

%translating closer to center
M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -fixedPC.ZLimits(2) 1];
tform = affine3d(M);
fixedPC = pctransform(fixedPC, tform); 
% show
%{
subplot(2, 4, 2);
pcshow(fixedPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
title("fixedPC centered");
%}

%removing noises
fixedPC = pcdenoise(fixedPC);

% add first pc to result
mergedPointClouds = copy(fixedPC);

for i = 2 : sizeNumbers
    
    %moving point cloud
    movingPC = pcread("data/framesTest3Limited/frameLimited" + num2str(numbersOfFrame(i)) + ".ply");
    %show
    %{
    subplot(2, 4, 5);
    pcshow(movingPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("movingPC");
    %}
    
    %translating closer to center
    M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -movingPC.ZLimits(2) 1];
    tform = affine3d(M);
    movingPC = pctransform(movingPC, tform); 
    % show
    %{
    subplot(2, 4, 2);
    pcshow(fixedPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("fixedPC centered");
    %}

    %removing noises
    movingPC = pcdenoise(movingPC);
    
    %getting downsamples
    fixedPCDownSample = pcdownsample(fixedPC, 'gridAverage', 0.0001);
    movingPCDownSample = pcdownsample(movingPC, 'gridAverage', 0.0001);
    mergedPointCloudsDownSample = pcdownsample(mergedPointClouds, 'gridAverage', 0.0001);
    
    %rigid trasformation ver1
    %
    [tform1,rmse1] = pcregistericp(movingPCDownSample, fixedPCDownSample, 'Verbose', true, 'Metric', 'pointToPlane', 'Extrapolate', true);
    tform = tform1;
    %
    
    %rigid trasformation ver2
    %{
    [tform2,rmse2] = pcregistericp(movingPCDownSampleLeft, mergedPointCloudsDownSample, 'Verbose', true, 'Metric', 'pointToPlane', 'Extrapolate', true);
    tform = tform2;
    %}
    
    % ver3
    %{
    if rmse1 < rmse2
        tform = tform1;
    else
        tform = tform2;
    end
    %}
    
    %tranforming
    pointCloudTransform = pctransform(movingPC, tform);
    %show
    %{
    subplot(2, 4, [3 7]);
    pcshow(pointCloudTransform, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("pointCloudTransform");
    %}
    
    %merging

    %merging pointcloud
    mergeSize = 0.0001;
    mergedPointClouds = pcmerge(mergedPointClouds, pointCloudTransform, mergeSize);
    %show
    %{
    subplot(2, 4, [4 8]);
    pcshow(mergedPointClouds, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("mergedPointClouds");
    %}

    %show
    figure;pcshow(mergedPointClouds);

    %setting new fixed point cloud to moving one
    fixedPC = pointCloudTransform;

end
