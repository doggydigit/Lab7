function [theta_out, r_out]=runCPGNetwork(f, phi_total, R, dt)
%%
%     input:
%         f - intrinsic frequency of oscillators in Hz (20 x 1)
%         R - nominal amplitudes in Hz (20 x 1)
%         dt - basic time step
%     output:
%         theta_out - CPG phase of size (20 x 1)
%         r_out - CPG radius of size (20 x 1)
%
%%  Initialization
    % define here static variables (keep their value over multiple calls of the
    % function). You can also put here your coupling weights etc...
    persistent is_init r theta t w;
    if isempty(is_init)
        is_init=true;
        theta=rand(20,1);
        r=zeros(20,1);
        w = 10*diag([ones(9,1);0;ones(9,1)],1)+diag([ones(9,1);0;ones(9,1)],-1)+diag(ones(10,1),10)+diag(ones(10,1),-10);
        % initialization of static/persistent variables (happens only
        % during first call)
        t=0;
    end   
    
    t=t+dt;

    %% HERE CALCULATE PHASE LAGS FROM phi_total (also amplitude gradient if needed)
    phi = phi_total*diag([ones(9,1)/10;0;ones(9,1)/10],1)+diag([ones(9,1)/10;0;ones(9,1)/10],-1)+diag(ones(10,1)/2,10)+diag(ones(10,1)/2,-10);
    
    %% HERE IMPLEMENT DOUBLE CHAIN OF OSCILLATORS REPRESENTING THE SPINAL CORD OF THE ROBOT
    dtheta = 2*pi*f + sum( sin(repmat(theta',20,1)-repmat(theta,1,20)-phi).*w.*repmat(r',20,1) ,2);
    a = 1;
    dr = a*(R-r);


    %% HERE IMPLEMENT EULER INTEGRATION
    theta = theta + dtheta*dt;
    r = r + dr*dt;

  
    
    %% Assign output
    theta_out=theta;
    r_out=r;

end
















