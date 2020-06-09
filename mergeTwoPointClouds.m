function error = mergeTwoPointClouds(pc1, pc2, variables)
    tic  % time
    
    % create matrix of transformation
    moveX = variables(1);
    moveY = variables(2);
    moveZ = variables(3);
    thetaX = variables(4);
    thetaY = variables(5);
    thetaZ = variables(6);
    MRotateX = [1 0 0 0;
        0 cos(thetaX) -sin(thetaX) 0;
        0 sin(thetaX) cos(thetaX) 0;
        0 0 0 1];
    MRotateY = [cos(thetaY) 0 sin(thetaY) 0;
        0 1 0 0;
        -sin(thetaY) 0 cos(thetaY) 0;
        0 0 0 1];
    MRotateZ = [cos(thetaZ) -sin(thetaZ) 0 0;
        sin(thetaZ) cos(thetaZ) 0 0;
        0 0 1 0;
        0 0 0 1];
    MTranslation = [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        moveX moveY moveZ 1];
    M = MRotateX * MRotateY * MRotateZ * MTranslation;
    
    % downsample
    gridStep = 0.01;
    pc1Opt = pcdownsample(pc1, 'gridAverage', gridStep);
    pc2Opt = pcdownsample(pc2, 'gridAverage', gridStep);
    
    % transform second point cloud
    tform = affine3d(M);
    pc2T = pctransform(pc2Opt,tform);

    % delete pc2 (up & down) - hair and feet
    roi = [pc2T.XLimits(1)-0.1 pc2T.XLimits(2)+0.1
        pc2T.YLimits(1) pc2T.YLimits(2)
        pc2T.ZLimits(1) pc2T.ZLimits(2)];
    indices = findPointsInROI(pc2T, roi);
    pc2T = select(pc2T, indices);

    % for calculations
    pc2Array = pc2T.Location; % transformed
    pc1PointCloud = pc1Opt; 
    s = size(pc2Array);
    error = 0;
    
    % calculations
    for i=1:s(1)
        [~, dist] = findNearestNeighbors(pc1PointCloud, pc2Array(i,:), 1);
        error = error + double(dist);
    end
    error = error / s(1);
    
    toc  % time
    disp(variables);
    disp(error);
end