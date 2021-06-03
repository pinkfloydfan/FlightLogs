clear
clc

bag = rosbag('good run 1.bag');
joystick = select(bag, 'Topic', 'joy_processed');
pose = select(bag, 'Topic', 'Mono_Inertial/orb_pose');


joymsgs = readMessages(joystick, 'DataFormat', 'struct');
joyTime = getRosTime(joymsgs);
axes = cellfun(@(m) double(m.Axes), joymsgs, 'UniformOutput', false)';

A = cell2mat(axes);
modeSwitch = A(6, :);
autopilotIndices = find(modeSwitch == -1);
autopilotEngageIndex = autopilotIndices(1); %index where modeSwitch first becomes -1
autopilotEngageTime = joyTime(autopilotEngageIndex);

posemsgs = readMessages(pose, 'DataFormat', 'struct');
poseTime = getRosTime(posemsgs);

xPoints = cellfun(@(m) double(m.Pose.Position.X),posemsgs);
yPoints = cellfun(@(m) double(m.Pose.Position.Y),posemsgs);
zPoints = cellfun(@(m) double(m.Pose.Position.Z),posemsgs);

% find this manually
endIndex = 1119;
endTime = poseTime(endIndex);

setpointIndex = findClosestIndex(autopilotEngageTime, poseTime);


poseTime_trimmed = poseTime(setpointIndex:endIndex);
xPoints_trimmed = xPoints(setpointIndex:endIndex);
yPoints_trimmed = yPoints(setpointIndex:endIndex);
zPoints_trimmed = zPoints(setpointIndex:endIndex);

x_setpoint = xPoints(setpointIndex);
y_setpoint = yPoints(setpointIndex);
z_setpoint = zPoints(setpointIndex);

setpoint_time = linspace(autopilotEngageTime, endTime, 2);

x_setpoint_vis = ones(1, 2)*x_setpoint;

y_setpoint_vis = ones(1, 2)*y_setpoint;

z_setpoint_vis = ones(1, 2)*z_setpoint;

%get the point of activation for the 

figure
plot3(xPoints_trimmed, yPoints_trimmed, zPoints_trimmed)
title('aircraft trajectory')


figure
plot(poseTime_trimmed, xPoints_trimmed)
title('x position')
xlabel('Time/s')
ylabel('Position/m')
hold on
plot(setpoint_time, x_setpoint_vis)

grid on 

figure
plot(poseTime_trimmed, yPoints_trimmed)
title('y position')
xlabel('Time/s')
ylabel('Position/m')
hold on
plot(setpoint_time, y_setpoint_vis)

grid on 


figure
plot(poseTime_trimmed, zPoints_trimmed)
title('z position')
xlabel('Time/s')
ylabel('Position/m')
hold on
plot(setpoint_time, z_setpoint_vis)

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