
function limitedPointCloudSaver(dirName, fileName, xLim, yLim, zLim) 

%number of files in dir
d = dir([convertStringsToChars(dirName) '/*.ply']);
numberOfFiles = size(d, 1);

%creating dir for limited point clouds
newDirName = dirName + "Limited";
if (~exist(newDirName, 'dir'))
    mkdir(newDirName);
end

for i = 1 : numberOfFiles
    
        %getting point cloud
        pointCloud = pcread(dirName + "/" + d(i).name);
        
        %region of interest
        roi = findPointsInROI(pointCloud, [xLim(1) xLim(2) yLim(1) yLim(2) zLim(1) zLim(2)]);
        pointCloud = select(pointCloud, roi);
   
        %saving limited point cloud to file
        filename = newDirName + "/" + fileName + "Limited" + num2str(i);
        pcwrite(pointCloud, filename, 'Encoding', 'binary');
    
end

end