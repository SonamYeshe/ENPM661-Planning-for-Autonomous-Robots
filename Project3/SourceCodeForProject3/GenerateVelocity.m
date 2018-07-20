function GenerateVelocity(pathX, pathY)
H = [eul2rotm([0 0 0]), [pathX(1), pathY(1), 0]'; [0 0 0 1]];
hz = 2;
resoluton = 0.05;
L = 0.23; % Axle distance.
r = 0.038; % Wheel radius.
vMax = 0.7;
wMax = 3.14;
fileID2 = fopen('Twist.txt', 'w');
fileID3 = fopen('WheelRotationalVelocity.txt', 'w');
for i = 2 : length(pathX)
    pNext = H \ [pathX(i) pathY(i) 0 1]';
    % Calculate the angular velocity along z axis.
    theta = atan2(pNext(2), pNext(1));
    vTheta = theta * hz;
    % Rotate the Turtlebot frame first due to the fact that it can only move along x axis.
    H = H * rotm2tform(eul2rotm([theta 0 0]));
    % Calculate the linear velocity alog x axis.
    pNext = H \ [pathX(i) pathY(i) 0 1]';
    vPNext = pNext(1) * hz * resoluton;
    % Translate the Turtlebot frame.
    H = H * trvec2tform(pNext(1:3)');
    % Restrict angular velocity according to the Turtlebot.
    countW = 1;
    while abs(vTheta) > wMax / 2
        vTheta = vTheta / 2;
        countW = countW * 2;
    end    
    % Follow mobile_base/commands/velocity to write down the results.
    % Wheel's rotational velocity as well.
    ur = vTheta * L / (2 * r);
    ul = -ur;
    formatSpec = 'linear.x = 0.0, linear.y = 0.0, linear.z = 0.0, angular.x = 0.0, angular.y = 0.0, angular.z = %f\n';
    formatSpec3 = 'Right wheel rotational velocity = %f, left wheel rotational velocity = %f\n';
    for j = 1 : countW
        fprintf(fileID2, formatSpec, vTheta);
        fprintf(fileID3, formatSpec3, ur, ul);
    end
    % Restrict linear velocity according to the Turtlebot.
    countV = 1;
    while abs(vPNext) > vMax / 2
        vPNext = vPNext / 2;
        countV = countV * 2;
    end
    % Follow mobile_base/commands/velocity to write down the linear velocity.
    % Wheel's rotational velocity as well.
    ur = vPNext / r;
    formatSpec2 = 'linear.x = %f, linear.y = 0.0, linear.z = 0.0, angular.x = 0.0, angular.y = 0.0, angular.z = 0.0\n';
    for j = 1 : countV
        fprintf(fileID2, formatSpec2, vPNext);
        fprintf(fileID3, formatSpec3, ur, ur);
    end
end
fclose(fileID2);
fclose(fileID3);
%%
% % loop1
% p1 = H \ [pathX(2) pathY(2) 0 1]';
% theta1 = atan2(p1(2),p1(1)); % theta1 is the delta_z
% vTheta1 = theta1 * hz; % vTheta1 is the w_z
% thrtaDegree1 = round(theta1/pi*180);
% H1 = H * rotm2tform(eul2rotm([theta1 0 0]));
% p1 = H1 \ [pathX(2) pathY(2) 0 1]'; % p1(1) is the delta_x
% vP1 = p1(1) * hz * resoluton; % vP1 is the v_x
% H1 = H1 * trvec2tform(p1(1:3)');
% 
% % loop2
% p2 = H1 \ [pathX(3) pathY(3) 0 1]';
% theta2 = atan2(p2(2),p2(1)); % theta2 is the delta_z
% vTheta2 = theta2 * hz; % vTheta2 is the w_z
% thrtaDegree2 = round(theta2/pi*180);
% H2 = H1 * rotm2tform(eul2rotm([theta2 0 0]));
% p2 = H2 \ [pathX(3) pathY(3) 0 1]'; % p2(1) is the delta_x
% vP2 = p2(1) * hz * resoluton; % vP1 is the v_x
% H2 = H2 * trvec2tform(p2(1:3)');