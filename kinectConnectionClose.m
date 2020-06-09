
function kinectConnectionClose(colorDevice, depthDevice)

    %releasing devices
    release(colorDevice);
    release(depthDevice);
    
end