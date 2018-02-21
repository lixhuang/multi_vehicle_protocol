function draw_st_road(xrange)

    if(~exist('road_image'))
        road_image=imread('drawlib/element/road.png');
    end
    w=3.7*3/2+0.2;
    l=3.7*2048/1500;

    i_l = xrange(1)/l-2;
    i_u = xrange(2)/l+2;

    for i=i_l:i_u
        imagesc([i*l,(i+1)*l],[-w,w],road_image);
    end

end