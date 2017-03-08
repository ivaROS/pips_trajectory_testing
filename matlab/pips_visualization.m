x = 0;
rosinit('localhost');
%gazebo = ExampleHelperGazeboCommunicator();

buffer = 1000;

if ismember('/camera/rgb/image_raw', rostopic('list'))
  imsub = rossubscriber('/camera/rgb/image_raw','BufferSize',buffer);
  depthsub = rossubscriber('/camera/depth/image_raw','BufferSize',buffer);
  %pathsub = rossubscriber('/GenAndTest/tested_paths');
  hallucsub = rossubscriber('/camera/depth/image_raw_pub','BufferSize',buffer);
  gendepthsub = rossubscriber('/generated_depth_image','BufferSize',buffer);
end

pause(1);

img = readImage(receive(imsub));
depthImg = readImage(receive(depthsub));

imwrite(img, '/home/justin/Documents/pub-pips/figs/hallucinated-robot/sequences/pose-9/rgb.png');
imwrite(depthImg, '/home/justin/Documents/pub-pips/figs/hallucinated-robot/sequences/pose-9/depth.png');

figure(1);
scdepth = imagesc(depthImg);
colorbar();
saveas(scdepth, '/home/justin/Documents/pub-pips/figs/hallucinated-robot/sequences/pose-9/depthsc.png');

while 1

%img = receive(imsub);
%figure(ind)
%imshow(readImage(img),'DisplayRange',[0,MAX_RANGE]);

MAX_RANGE = 8;


genImg = readImage(receive(gendepthsub));
hallucImg = readImage(receive(hallucsub));

filenumber = sprintf('/home/justin/Documents/pub-pips/figs/hallucinated-robot/sequences/pose-9/%03d', x);

x = x + 1;

imwrite(genImg, [filenumber, 'generated.png']);
imwrite(hallucImg, [filenumber, 'hallucinated.png']);

figure(1);
imshow(genImg,'DisplayRange',[0,MAX_RANGE]);


sc = imagesc(genImg,[min(min(depthImg)), max(max(depthImg))]);




% figure(1);
% imshow(depthImg,'DisplayRange',[0,MAX_RANGE]);
% 
% figure(2);
% imshow(genImg,'DisplayRange',[0,MAX_RANGE]);
% 
% figure(3);
% imshow(hallucImg);

%path = receive(pathsub);

end
% 
% 
% pause(2);
% 
% 
% clear;
% 
% 
% rosshutdown;