
function frameGrabber(dirName, fileName, numberOfFrames)
       
    %initializing connection
    [colorDevice, depthDevice] = kinectConnectionInit();
    
    %saving point cloud to file
    if (~exist(dirName, 'dir'))
        mkdir(dirName);
    end
    
    %grabbing frames
    for i = 1 : numberOfFrames
    
        %loading frame from device
        colorFrame = step(colorDevice);
        depthFrame = step(depthDevice);
    
        %extracting point cloud
        pointCloud = pcfromkinect(depthDevice, depthFrame, colorFrame);   
    
        filename = dirName + "/" + fileName + num2str(i);
        pcwrite(pointCloud, filename, 'Encoding', 'binary');  
    
    end

    kinectConnectionClose(colorDevice, depthDevice);
    
end



