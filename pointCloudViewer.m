
function pointCloudViewer(dirName, fileName, varargin)

    %reading point cloud from a file  
    filename = dirName + "/" + fileName + ".ply";
    pointCloud = pcread(filename);
    
    %limiting axis
    if (nargin == 3) 
       xLim = varargin{1}; 
       yLim = pointCloud.YLimits;
       zLim = pointCloud.ZLimits;
    elseif (nargin == 4)
       xLim = varargin{1};
       yLim = varargin{2};
       zLim = pointCloud.ZLimits;
    elseif (nargin == 5)
       xLim = varargin{1};
       yLim = varargin{2};
       zLim = varargin{3};
    else
       xLim = pointCloud.XLimits;
       yLim = pointCloud.YLimits;
       zLim = pointCloud.ZLimits;
    end
    
    %viewing point cloud
    pcshow(pointCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
    title(filename);
    xlim(xLim); 
    ylim(yLim);
    zlim(zLim); 
    xlabel("x");
    ylabel("y");
    zlabel("z");
    
end