%% merging from two sides
%% Section 1 - chosen frames

%numbers of chosen frames
numbersOfFrame = [4 6 16 71 21 26 29 32 39 47 52 59 56 63]; 
sizeNumbers = size(numbersOfFrame, 2);


%viewing chosen frames
%
figure;
for i = 1 : sizeNumbers
   subplot(2, ceil(sizeNumbers / 2), i);
   pcshow(pcread("data/framesTest3Limited/frameLimited" + num2str(numbersOfFrame(i)) + ".ply"), 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
end
%

%% Section 2 - merging 

%fixed point cloud
fixedPCLeft = pcread("data/framesTest3Limited/frameLimited" + num2str(numbersOfFrame(1)) + ".ply");

%translating closer to center
M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -fixedPCLeft.ZLimits(2) 1];
tformLeft = affine3d(M);
fixedPCLeft = pctransform(fixedPCLeft, tformLeft); 
% show
%{
subplot(2, 4, 2);
pcshow(fixedPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
title("fixedPC centered");
%}

%removing noises
fixedPCLeft = pcdenoise(fixedPCLeft);

% add first pc to result
mergedPointClouds = copy(fixedPCLeft);
fixedPCRight = copy(fixedPCLeft);

for i = 1 : (sizeNumbers/2)
    
    %moving point cloud
    movingPCLeft = pcread("data/framesTest3Limited/frameLimited" + num2str(numbersOfFrame(i+1)) + ".ply");
    movingPCRight = pcread("data/framesTest3Limited/frameLimited" + num2str(numbersOfFrame(sizeNumbers+1 - i)) + ".ply");
    %show
    %{
    subplot(2, 4, 5);
    pcshow(movingPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("movingPC");
    %}
    
    %translating closer to center
    M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -movingPCLeft.ZLimits(2) 1];
    tformLeft = affine3d(M);
    movingPCLeft = pctransform(movingPCLeft, tformLeft); 
    M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -movingPCRight.ZLimits(2) 1];
    tformRight = affine3d(M);
    movingPCRight = pctransform(movingPCRight, tformRight); 
    % show
    %{
    subplot(2, 4, 2);
    pcshow(fixedPC, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("fixedPC centered");
    %}

    %removing noises
    movingPCLeft = pcdenoise(movingPCLeft);
    movingPCRight = pcdenoise(movingPCRight);
    
    %getting downsamples
    fixedPCDownSampleLeft = pcdownsample(fixedPCLeft, 'gridAverage', 0.0001);
    fixedPCDownSampleRight = pcdownsample(fixedPCRight, 'gridAverage', 0.0001);
    movingPCDownSampleLeft = pcdownsample(movingPCLeft, 'gridAverage', 0.0001);
    movingPCDownSampleRight = pcdownsample(movingPCRight, 'gridAverage', 0.0001);
    mergedPointCloudsDownSample = pcdownsample(mergedPointClouds, 'gridAverage', 0.0001);
    
    %rigid trasformation ver1
    %
    [tformLeft1,rmseL1] = pcregistericp(movingPCDownSampleLeft, fixedPCDownSampleLeft, 'Verbose', true, 'Metric', 'pointToPlane', 'Extrapolate', true);
    [tformRight1,rmseR1] = pcregistericp(movingPCDownSampleRight, fixedPCDownSampleRight, 'Verbose', true, 'Metric', 'pointToPlane', 'Extrapolate', true);
    tformLeft = tformLeft1;
    tformRight = tformRight1;
    %
    
    %rigid trasformation ver2
    %{
    [tformLeft2,rmseL2] = pcregistericp(movingPCDownSampleLeft, mergedPointCloudsDownSample, 'Verbose', true, 'Metric', 'pointToPlane', 'Extrapolate', true);
    [tformRight2,rmseR2] = pcregistericp(movingPCDownSampleRight, mergedPointCloudsDownSample, 'Verbose', true, 'Metric', 'pointToPlane', 'Extrapolate', true);
    tformLeft = tformLeft2;
    tformRight = tformRight2;
    %}
    
    % ver3
    %{
    if rmseL1 < rmseL2
        tformLeft = tformLeft1;
    else
        tformLeft = tformLeft2;
    end
    if rmseR1 < rmseR2
        tformRight = tformRight1;
    else
        tformRight = tformRight2;
    end
    %}
    
    %tranforming
    pointCloudTransformLeft = pctransform(movingPCLeft, tformLeft);
    pointCloudTransformRight = pctransform(movingPCRight, tformRight);
    %show
    %{
    subplot(2, 4, [3 7]);
    pcshow(pointCloudTransform, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("pointCloudTransform");
    %}
    
    %merging

    %merging pointcloud left
    mergeSize = 0.0001;
    mergedPointClouds = pcmerge(mergedPointClouds, pointCloudTransformLeft, mergeSize);
    %show
    %{
    subplot(2, 4, [4 8]);
    pcshow(mergedPointClouds, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("mergedPointClouds");
    %}

    %merging pointcloud right
    mergedPointClouds = pcmerge(mergedPointClouds, pointCloudTransformRight, mergeSize);
    %show
    %{
    subplot(2, 4, [4 8]);
    pcshow(mergedPointClouds, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title("mergedPointClouds");
    %}


    
    %show
    figure;pcshow(mergedPointClouds);

    %setting new fixed point cloud to moving one
    fixedPCLeft = pointCloudTransformLeft;
    fixedPCRight = pointCloudTransformRight;

end
