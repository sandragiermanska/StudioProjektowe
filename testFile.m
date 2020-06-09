%clearing stuff
close all;
clc;
clearvars;

%pause(10);

%grabbing frames
%frameGrabber("data/framesTest4", "frame", 80);

%pointCloudViewer("data/framesTest3", "frame1", [-0.3 0.7], [-1.5 1.04], [1.8 2.8]);

%viewing all frames from given dir
% licznik = 1;
% for i = 1 : 5
%     figure;
%     for j = 1 : 4
%        subplot(2, 2, j);
%        pointCloudViewer("data/frames10", "frame" + num2str(licznik), [-0.5 0.5], [-0.3 0.4], [0.2 1.8]);
%        licznik = licznik + 1;
%     end
% end

%saving limited point clouds
%limitedPointCloudSaver("data/framesTest3", "frame", [-0.3 0.7], [-1.5 1.04], [1.8 2.8]);

%viewing all limited frames from given dir
% licznik = 1;
% for i = 1 : 5
%     figure;
%     for j = 1 : 4
%        subplot(2, 2, j);
%        pointCloudViewer("data/framesTest1Limited", "frameLimited" + num2str(licznik));
%        licznik= licznik + 1;
%     end
% end

% figure;
% for i = 1 : 10
%        subplot(2, 5, i);
%        pointCloudViewer("data/framesTest2Limited", "frameLimited" + num2str(i));
% end
% 
% figure;
% for i = 10 : 20
%        subplot(2, 5, mod(i, 10) + 1);
%        pointCloudViewer("data/framesTest2Limited", "frameLimited" + num2str(i));
% end
% 
% figure;
% for i = 20 : 30
%        subplot(2, 5, mod(i, 10) + 1);
%        pointCloudViewer("data/framesTest2Limited", "frameLimited" + num2str(i));
% end
% 
% for j = 0 : 7
%     figure;
%     for i = 1 : 10
%        subplot(2, 5, mod(j * 10 + i, 10) + 1);
%        pointCloudViewer("data/framesTest3Limited", "frameLimited" + num2str(j * 10 + i));
%     end
% end

 
