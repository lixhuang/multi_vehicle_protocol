function draw_car_grey(x,y,theta,v)
    w=2.4;
    l=4.5;
    if(~exist('grey_car_image'))
        grey_car_image = imread('drawlib/element/grey_car.png');
    end
    if(nargin==3)
        v=l/2;
    end
    imagesc([x-l/2,x+l/2],[y-w/2,y+w/2],grey_car_image);
    quiver(x,y,v*cos(theta),v*sin(theta), 'LineWidth', 2, 'MaxHeadSize', 1);
end

