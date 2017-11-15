if(~exist('road_image'))
    road_image=imread('drawlib/element/road.png');
end
w=3.7*3/2+0.2;
l=3.7*2048/1500;
for i=-2:7
    imagesc([i*l,(i+1)*l],[-w,w],road_image);
end