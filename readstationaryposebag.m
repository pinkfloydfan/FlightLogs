clear
clc

bag = rosbag('long stationary.bag');
pose = select(bag, 'Topic', 'Mono_Inertial/orb_pose');


posemsgs = readMessages(pose, 'DataFormat', 'struct');
poseTime = getRosTime(posemsgs);

xPoints = cellfun(@(m) double(m.Pose.Position.X),posemsgs);
yPoints = cellfun(@(m) double(m.Pose.Position.Y),posemsgs);
zPoints = cellfun(@(m) double(m.Pose.Position.Z),posemsgs);

figure
plot(xPoints)

figure
plot(yPoints)

figure
plot(zPoints)

% find this manually
startIndex = 2015;

endIndex = 8220;
endTime = poseTime(endIndex);



poseTime_trimmed = poseTime(startIndex:endIndex);
xPoints_trimmed = xPoints(startIndex:endIndex);
yPoints_trimmed = yPoints(startIndex:endIndex);
zPoints_trimmed = zPoints(startIndex:endIndex);


%{
setpointIndex = findClosestIndex(autopilotEngageTime, poseTime);

x_setpoint = xPoints(setpointIndex);
y_setpoint = yPoints(setpointIndex);
z_setpoint = zPoints(setpointIndex);

setpoint_time = linspace(autopilotEngageTime, endTime, 2);

x_setpoint_vis = ones(1, 2)*x_setpoint;

y_setpoint_vis = ones(1, 2)*y_setpoint;

z_setpoint_vis = ones(1, 2)*z_setpoint;
%}

%get the point of activation for the 

figure
plot3(xPoints_trimmed, yPoints_trimmed, zPoints_trimmed)
title('aircraft trajectory')


figure
plot(poseTime_trimmed, xPoints_trimmed)
title('x position')
xlabel('Time/s')
ylabel('Position/m')
%hold on
%plot(setpoint_time, x_setpoint_vis)

grid on 

figure
plot(poseTime_trimmed, yPoints_trimmed)
title('y position')
xlabel('Time/s')
ylabel('Position/m')
%hold on
%plot(setpoint_time, y_setpoint_vis)

grid on 


figure
plot(poseTime_trimmed, zPoints_trimmed)
title('z position')
xlabel('Time/s')
ylabel('Position/m')
%hold on
%plot(setpoint_time, z_setpoint_vis)

grid on 

pdx = fitdist(xPoints_trimmed, 'Normal')
pdy = fitdist(yPoints_trimmed, 'Normal')
pdz = fitdist(zPoints_trimmed, 'Normal')

figure
histx = histfit(rmoutliers(xPoints_trimmed)-pdx.mu)
title("X-position estimate")
xlabel("Position/10^{-1}m")
ylabel("Frequency")
xlim([-1.5e-3 1.5e-3])
grid on

figure
histy = histfit(rmoutliers(yPoints_trimmed)-pdy.mu)
title("Y-position estimate")
xlabel("Position/10^{-1}m")
ylabel("Frequency")
xlim([-1.5e-3 1.5e-3])
grid on


figure
histz = histfit(rmoutliers(zPoints_trimmed)-pdz.mu)
title("Z-position estimate")
xlabel("Position/10^{-1}m")
ylabel("Frequency")

xlim([-1.5e-3 1.5e-3])
grid on



function sequentialTime = getRosTime(messages)
    time_sec = cellfun(@(m) double(m.Header.Stamp.Sec), messages);
    time_nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), messages);

    time_normalised = time_sec - time_sec(1);
    sequentialTime  = time_normalised + time_nsec*1e-9;
end

function index = findClosestIndex(val, array)
    [d, ix] = min(abs(val-array));
    index = ix;
end 