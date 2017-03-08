%rosinit('localhost');

x = 0;

buffer = 1000;

if ismember('/model_depth', rostopic('list'))
  gendepthsub = rossubscriber('/model_depth','BufferSize',buffer);
end

pause(1);


while 1

%img = receive(imsub);
%figure(ind)
%imshow(readImage(img),'DisplayRange',[0,MAX_RANGE]);

MAX_RANGE = 8;


genImg = readImage(receive(gendepthsub));

maxval = max(max(genImg));
minval = min(min(genImg(genImg > 0)));


figure(1);
imshow(genImg,'DisplayRange',[0,MAX_RANGE]);

if(maxval > 0)
    genImgScaled = 1/(maxval - minval)  * (genImg - minval);
    genImgScaled(genImg == 0) = (maxval - minval)/2;
    
    figure(2);
    imshow(genImgScaled);
    
    figure(3);
    clims = [minval maxval];
    imagesc(genImg, clims);
    colorbar()
    
else
    figure(3);
    imagesc(genImg);
    colorbar()

end


    
    

    x = x + 1


end
