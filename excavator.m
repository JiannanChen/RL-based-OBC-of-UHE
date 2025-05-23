%{
This code is the solution for the flatness of the excavator.
The expected values of each joint Angle and the hydraulic cylinder can be solved from the flatness trajectory.
%}
%The specific ADP control algorithm can be referred to：https://github.com/JiannanChen/RL-basedOBC-of-UFFR.git
clear
pi = 180;
i = 1;
    %Vehicle chassis Angle
    phi_pitchup = 0;%Elevation Angle
    phi_pitchdown = 0;%Lower pitch Angle
    phi_leftincline = 0;%Left inclination Angle
    phi_rightincline = 0;%Right tilt Angle
    
    %Parameters of the excavator
    %{
     The excavator corresponding to this code is a small excavator, 
     and its size is not consistent with that in the paper
    %}
    L01 = 0.87;
    L02 = 0.13;
    L1 = 3.375;
    L2 = 2.1;
    L3 = 1.04522;
    X0 = 0.2;
    H0 = 1.491;
    GN = 1.45675;
    MN = 0.395;
    QN = 0.2584;
    QK = 0.312;
    MK = 0.342;
    PJ = 1.50296;
    JW = 0.4;
    PW0 = 1.48832;
    AC = 0.479513;
    BC = 1.73023;
    DF = 1.7535;
    EF = 0.50812;
    d_boom0 = 1.284;
    d_arm0 = 1.328;
    d_bucket0 = 1.1;
    CD = 2.30175;
    EG = 0.857833;
    FG = 0.564447;
    FN = 1.84242;
    VK = 1.17876;
    QI = 0.690018;
    VI = 0.985652;
    QR = 0.782468;
    RV = 0.649934;
    W_bucket = 0.77;
    FNG = 14.452;
    FNQ = 175.116;
    FQN = 4.283;
    VQK = 107.826;
    BCF = 23.703;
    ACT1 = 54.918;
    ACT2 = 35.082;
    DFC = 39.2384;
    EFQ = 146.8;
    CDF = 111.953; 
    DCF = 39.2384;
    GEF = 39.211;
    QFG = 40.698;
    FGN = 125.45;
    QFN = 0.6;
    QIT6 = 35.2794;
    KQI = 42.2794;
    IQV = 64.5469;
    %The initial attitude of the bucket obtained through the kinematic positive solution (based on the world coordinate system)
    %Vehicle body corner theta1
    theta1_0 = 0;
    
    %boom Angle theta2
    theta2_0 = 0;
  

    %The position based on the vehicle coordinate system 
    x_vehicle0 = 0;y_vehicle0 = 5;z_vehicle0 = 0.5;

    %Transformation based on the world coordinate system
    %The vehicle's upward tilt matrix
    R_pitchup = [1,0,0;0,cosd(phi_pitchup),-sind(phi_pitchup);0,sind(phi_pitchup),cosd(phi_pitchup)];
    %Vehicle downward-bending matrix
    R_pitchdown = [1,0,0;0,cosd(abs(phi_pitchdown)),sind(abs(phi_pitchdown));0,-sind(abs(phi_pitchdown)),cosd(abs(phi_pitchdown))];
    %The left-tilt matrix of the entire vehicle
    R_leftincline = [cosd(abs(phi_leftincline)),0,-sind(abs(phi_leftincline));0,1,0;sind(abs(phi_leftincline)),0,cosd(abs(phi_leftincline))];
    %The right-leaning matrix of the entire vehicle
    R_rightincline = [cosd(phi_rightincline),0,sind(phi_rightincline);0,1,0;0,-sind(phi_rightincline),cosd(phi_rightincline)];
    %Vehicle rotation matrix
    R_correction = R_pitchup*R_pitchdown*R_leftincline*R_rightincline;
    x0 = R_correction(1,:)*[x_vehicle0;y_vehicle0;z_vehicle0];
    y0 = R_correction(2,:)*[x_vehicle0;y_vehicle0;z_vehicle0];
    z0 = R_correction(3,:)*[x_vehicle0;y_vehicle0;z_vehicle0];


    %slope angle
    theta_slope = -10;
    %surface height
    l_deep = 0;
    %The nearest safe distance to the front of the entire machine yf
    yf = 2;
    %When leveling the ground zf=z0
    zf = -tand(theta_slope)*yf+z0+l_deep+y0*tand(theta_slope);
    
    %velocity
    v = -0.1;

    %Calculate working hours
    tb = 10;
    a = -v/(tb^2);
    b = 2*v/tb;
    tf = (yf-y0+v*tb*2/3)/v;
 
    for t = 0:0.05:tf
        %The bucket keeps moving horizontally with the world coordinate system
        if t<=tb%Acceleration section
        y(i) = y0+a*(t^3)/3+b*(t^2)/2;
        else if t<=tf-tb && t>tb%Uniform speed section
        y(i) = y0+a*(tb^3)/3+b*(tb^2)/2+v*(t-tb);         
        else if t>tf-tb%Deceleration section
        y(i)= y0+a*(tb^3)/3+b*(tb^2)/2+v*(tf-2*tb)+...
        a*(t^3-(tf-tb)^3)/3-(a*tf+b/2)*(t^2-(tf-tb)^2)+(a*(tf^2)+b*tf)*(t-tf+tb);
        end
        end
        end

    %The X-axis remains stationary
    x(i) = x0;

    %Z-axis (When leveling the ground zf=z0, z remains unchanged)
    v_z=(zf-z0)/(tf-tb*2/3);
    a_z=-v_z/(tb^2);
    b_z=2*v_z/tb;
    if t<=tb%Acceleration section
    z(i) = z0+a_z*(t^3)/3+b_z*(t^2)/2;
    else if t<=tf-tb && t>tb%Uniform speed section
    z(i) = z0+a_z*(tb^3)/3+b_z*(tb^2)/2+v_z*(t-tb);         
    else if t>tf-tb%Deceleration section
    z(i)= z0+a_z*(tb^3)/3+b_z*(tb^2)/2+v_z*(tf-2*tb)+...
    a_z*(t^3-(tf-tb)^3)/3-(a_z*tf+b_z/2)*(t^2-(tf-tb)^2)+(a_z*(tf^2)+b_z*tf)*(t-tf+tb);
    end
    end
    end

    %Vehicle body corner theta1
    theta1(i) = theta1_0;
    %Vehicle body corner theta2
    theta2(i) = theta2_0;

    %gamma is obtained through the sensor and remains constant
    gamma(i) = -90;

    Lcv(i) = sqrt((y(i)-(L01+L02))^2 + (H0-z(i))^2);
    Lfv(i) = sqrt(L2^2+L3^2-2*L2*L3*cosd(gamma(i)));

    %Attitude solution (inverse kinematics solution）
    alpha(i) = acosd((L1^2+Lcv(i)^2-Lfv(i)^2)/(2*L1*Lcv(i)))-atand((H0-z(i))/(y(i)-(L01+L02)));

    beta(i) = acosd((L1^2+Lfv(i)^2-Lcv(i)^2)/(2*L1*Lfv(i)))+acosd((L2^2+Lfv(i)^2-L3^2)/(2*L2*Lfv(i)))-pi;

    % Calculate the displacement of the oil cylinder from the joint Angle
    d_boom(i) = sqrt(AC^2+BC^2-2*AC*BC*cosd(BCF+alpha(i)+ACT1))-d_boom0;
    d_arm(i) = sqrt(DF^2+EF^2-2*DF*EF*cosd(pi-beta(i)-DFC-EFQ))-d_arm0;

    if gamma(i)>=-VQK-FQN
        KQN(i) = pi-FQN-VQK-gamma(i);
        NK(i) = sqrt(QN^2+QK^2-2*QN*QK*cosd(KQN(i)));
        QNM(i) = acosd((QN^2+NK(i).^2-QK^2)/(2*QN*NK(i)))+acosd((MN^2+NK(i).^2-MK^2)/(2*MN*NK(i)));
        d_bucket(i)=sqrt(GN^2+MN^2-2*GN*MN*cosd(2*pi-FNG-FNQ-QNM(i)));
    else
        KQN(i) = pi+FQN+VQK+gamma(i);
        NK(i) = sqrt(QN^2+QK^2-2*QN*QK*cosd(KQN(i)));
        QNM(i) = -acosd((QN^2+NK(i).^2-QK^2)/(2*QN*NK(i)))+acosd((MN^2+NK(i).^2-MK^2)/(2*MN*NK(i)));
        d_bucket(i)=sqrt(GN^2+MN^2-2*GN*MN*cosd(2*pi-FNG-FNQ-QNM(i)));
    end   
    i=i+1;
end
t = 0:0.05:tf;
N=tf/0.05;
th1=0;
yd(1:N+1) = L01*cosd(theta1(1:N+1))+cosd(theta1(1:N+1)+theta2(1:N+1)).*(L3*cosd(alpha(1:N+1)+beta(1:N+1)+gamma(1:N+1))+L2*cosd(alpha(1:N+1)+beta(1:N+1))+L1*cosd(alpha(1:N+1))+L02);
zd(1:N+1) = L3*sind(alpha(1:N+1)+beta(1:N+1)+gamma(1:N+1))+L2*sind(alpha(1:N+1)+beta(1:N+1))+L1*sind(alpha(1:N+1))+H0;

%Angular velocity and displacement velocity
omega_alpha(1:N) = (alpha(2:N+1)-alpha(1:N))/0.05;
omega_beta(1:N) = (beta(2:N+1)-beta(1:N))/0.05;
omega_gamma(1:N) = (gamma(2:N+1)-gamma(1:N))/0.05;
v_boom = (d_boom(2:N+1)-d_boom(1:N))/0.05;
v_arm = (d_arm(2:N+1)-d_arm(1:N))/0.05;
v_bucket = (d_bucket(2:N+1)-d_bucket(1:N))/0.05;

figure(1)
plot(t,alpha,'r',t,beta,'g',t,gamma,'b','LineWidth',2);
title('theta');
legend('\alpha','\beta','\gamma');
figure(2);
plot(t,d_boom,'r',t,d_arm,'g',t,d_bucket,'b','LineWidth',2);
title('Cylinder displacement');
legend('d\_boom','d\_arm','d\_bucket');
figure(3);
plot(t,zd,'LineWidth',2);
title('zd');
ylim([-1,1]);
figure(4);
plot(t,yd,'LineWidth',2);
title('yd');
figure(5);
plot(yd,zd,'r','LineWidth',2);hold on
xlim([2,5.5])
ylim([-1,1])
title('The trajectory of the bucket tip');
figure(6)
plot(t(1:N),omega_alpha,'r',t(1:N),omega_beta,'g',t(1:N),omega_gamma,'b','LineWidth',2);
title('Joint angular velocity');
legend('\omega\_\alpha','\omega\_\beta','\omega\_\gamma');
figure(7);
plot(t(1:N),v_boom,'r',t(1:N),v_arm,'g',t(1:N),v_bucket,'b','LineWidth',2);
title('Cylinder speed');
legend('v\_boom','v\_arm','v\_bucket');
