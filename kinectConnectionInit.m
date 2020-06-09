
function [colorDevice, depthDevice] = kinectConnectionInit()

    %defining object for color device
    colorDevice = imaq.VideoDevice('kinect', 1);

    %defining object for depth device
    depthDevice = imaq.VideoDevice('kinect', 2);

    %initializing camera 
    step(colorDevice);
    step(depthDevice);
    
end