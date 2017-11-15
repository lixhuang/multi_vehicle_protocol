function draw_car_red(x,y,theta,v)
    w=2.4;
    l=4.5;
    if(~exist('red_car_image'))
        red_car_image = imread('drawlib/element/red_car.png');
    end
    if(nargin==3)
        v=l/2;
    end
    imagesc([x-l/2,x+l/2],[y-w/2,y+w/2],red_car_image);
    quiver(x,y,v*cos(theta),v*sin(theta));
end

