function pointCloudResult = createModel(dirname, filename, numbersOfFrame)

    thetaX = 0;
    thetaY = pi/4;
    thetaZ = -0.15;
    moveX = -0.2;
    moveY = -0.02;
    moveZ = 0;

    variables = [moveX moveY moveZ thetaX thetaY thetaZ];

    % read first point cloud
    pointCloud1 = pcread(dirname + filename + num2str(numbersOfFrame(1)) + ".ply");
    
    % translation - closer to the center
    M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -pointCloud1.ZLimits(2) 1];
    tform = affine3d(M);
    pointCloud1 = pctransform(pointCloud1,tform);
    
    % add first pc to result
    pointCloudResult = copy(pointCloud1);
    
    % option in fmincon
    option = optimoptions('fmincon','TolX', 1e-4, 'TolFun', 1e-4);
    
    s = size(numbersOfFrame);
    for i = 2:s(2)
        % read second point cloud
        pointCloud2 = pcread(dirname + filename + num2str(numbersOfFrame(i)) + ".ply");
        
        % translation - closer to the center
        M = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -pointCloud2.ZLimits(2) 1];
        tform = affine3d(M);
        pointCloud2 = pctransform(pointCloud2,tform);
    
        % find transform matrix for second pc
        variables = fmincon(@(var) mergeTwoPointClouds(pointCloud1, pointCloud2, var), variables,[],[],[],[],[-Inf -Inf -Inf -Inf pi/6 -Inf], [Inf Inf Inf Inf pi/3 Inf], [], option);
        
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

        
        % use transform matrix on second pc
        tform = affine3d(M);
        pc2 = pctransform(pointCloud2,tform);
        
        % merge result and transformated pc
        pointCloudResult = pcmerge(pointCloudResult, pc2, 0.001);
        
        % transform result to observationPoint like in last merged pc
        tform = affine3d(inv(M));
        pointCloudResult = pctransform(pointCloudResult, tform);
        
        figure;pcshow(pointCloudResult);
        
        % update first point cloud
        pointCloud1 = pointCloud2;
        
        disp('connect frames');

    end


end