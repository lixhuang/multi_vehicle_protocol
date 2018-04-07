 function [vec, dtheta] = merge_vector_field(targets, q, env)
    % work for circular object
    % q = [x, y, theta, v]
    % repulsive funcrtion distance
    ob_sz = env.car_w;
    sep_min = env.min_sep;
    sig_d = 5.7;
    f = 0;
    sz = size(targets);
    
    xbar = q(1);
    ybar = q(2);
    dxbar = q(4)*cos(q(3));
    dybar = q(4)*sin(q(3));
    fx = 0;
    fy = 0;
    sigma = 1;
    dsigma = 0;
    
    for i = 1:sz(2)
        d_vec = [xbar-targets(1,i); ybar-targets(2,i)];
        d = sqrt(sum(d_vec.^2));
        if(d >= sig_d)
            continue;
        end
        if(f == 1)
            show("overlap obstacle error");
        end
        f = 1;
        
        xt = targets(1,i);
        yt = targets(2,i);
        dxt = targets(4,i)*cos(targets(3,i));
        dyt = targets(4,i)*sin(targets(3,i));
        
        x = d_vec(1);
        y = d_vec(2);
        dx = dxbar - dxt;
        dy = dybar - dyt;
        
        px = xt/sqrt(xt^2+yt^2);
        py = yt/sqrt(xt^2+yt^2);    
        dpx = dxt/sqrt(xt^2+yt^2)-(xt*dxt+yt*dyt)*xt/(xt^2+yt^2)^(3/2);
        dpy = dyt/sqrt(xt^2+yt^2)-(xt*dxt+yt*dyt)*yt/(xt^2+yt^2)^(3/2);
        
        if(d_vec'*[px;py]>0)
            l = 1;
        else
            l = 0;
        end
        
        fx = (l-1)*px*x^2+l*py*x*y-px*y^2;
        fy = (l-1)*py*y^2+l*px*x*y-py*x^2;
        fxn = fx/sqrt(fx^2+fy^2);
        fyn = fy/sqrt(fx^2+fy^2);
        
        dfx = 2*(l-1)*x*px*dx+l*py*(x*dy+y*dx)-2*px*y*dy+(l-1)*x^2*dpx+l*x*y*dpy-y^2*dpx;
        dfy = 2*(l-1)*y*py*dy+l*px*(x*dy+y*dx)-2*py*x*dx+(l-1)*y^2*dpy+l*x*y*dpx-x^2*dpy;
        dfxn = dfx/sqrt(fx^2+fy^2)-(fx*dfx+fy*dfy)*fx/(fx^2+fy^2)^(3/2);
        dfyn = dfy/sqrt(fx^2+fy^2)-(fx*dfx+fy*dfy)*fy/(fx^2+fy^2)^(3/2);
        
        beta = ob_sz^2-sum(d_vec.^2);
        dbeta = -2*x*dx-2*y*dy;
        beta_in = ob_sz^2-(ob_sz+sep_min+ob_sz)^2;
        beta_out = ob_sz^2-sig_d^2;
        if(beta>beta_in)
            vec = [0;0];
            dtheta = 0;
            return
        end
        cof_mat = [beta_out^3,beta_out^2,beta_out,1;
            3*beta_out^2,2*beta_out,1,0;
            beta_in^3,beta_in^2,beta_in^1,1;
            3*beta_in^2,2*beta_in,1,0];
        coff = cof_mat^-1*[1;0;0;0];
        sigma = [beta^3,beta^2,beta,1]*coff;
        dsigma = [3*beta^2,2*beta,1,0]*coff*dbeta;
    end
    fxa = -xbar;
    fya = -ybar;
    if(ybar==0)
        fxa = 0;
        fya = 0; 
    end
    dfxa = -dxbar;
    dfya = -dybar;
    
    
    fxan = fxa/sqrt(fxa^2+fya^2);
    fyan = fya/sqrt(fxa^2+fya^2);
    dfxan = dfxa/sqrt(fxa^2+fya^2)-(fxa*dfxa+fya*dfya)*fxa/(fxa^2+fya^2)^(3/2);
    dfyan = dfya/sqrt(fxa^2+fya^2)-(fxa*dfxa+fya*dfya)*fya/(fxa^2+fya^2)^(3/2);
    
    if(fx^2+fy^2==0)
        fxn = 0;
        fyn = 0;
        dfxn = 0;
        dfyn = 0;
    end
    
    Fx = sigma*fxan + (1-sigma)*fxn;
    Fy = sigma*fyan + (1-sigma)*fyn;
    dFx = sigma*dfxan + (1-sigma)*dfxn + dsigma*fxan - dsigma*fxn;
    dFy = sigma*dfyan + (1-sigma)*dfyn + dsigma*fyan - dsigma*fyn;
    
    
    vec = [Fx;Fy];
    vec = vec./sqrt(sum(vec.^2));
    
    dtheta = (Fy*dFx-Fx*dFy)/(Fx^2+Fy^2);
end

