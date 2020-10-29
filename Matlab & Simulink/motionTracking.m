%setup
clc
clear all

%user defined variables
fileName = 'test.mp4';
span = 0.1;
samplingSize = 1;

%do not change
addpath('Videos');
folder = fileparts(which(fileName));
xAxis = 1;
header1 = 'Position vs Time';
xLabel = 'Time (Seconds)';
yLabel1 = 'Position (Degrees)';
header2 = 'Velocity vs Frame Count';
yLabel2 = 'Angular Velocity (Degrees/Seconds)';
header3 = 'Acceleration vs Frame Count';
yLabel3 = 'Angular Acceleration (Degrees/Seconds^2)';


%get video
if ~exist(fileName, 'file')
    disp("The file " + fileName + " does not exist in this folder. Please try again")
else
    disp("The file " + fileName + " has been successfully targeted.")   
end
videoObject = VideoReader(fileName); %create video object
numFrames = videoObject.numFrames; %get number of frames
frameRate = videoObject.FrameRate;

for i = 1:samplingSize:numFrames
    frame = read(videoObject, i);
    %get channels
    r = frame(:,:,1);
    g = frame(:,:,2);
    b = frame(:,:,3);

    %create masks
    highTHoldR = 140;
    highTHold = 130;
    lowTHoldR = 130;
    lowTHold = 120;
    rmask = r>highTHoldR & g<lowTHold & b<lowTHold;
    gmask = g>highTHold & b<lowTHold & r<lowTHold;
    bmask = b>highTHold & g<lowTHold & r<lowTHold;
    r(~rmask) = 0;
    r(rmask) = 255;
    g(~gmask) = 0;
    g(gmask) = 255;
    b(~bmask) = 0;
    b(bmask) = 255;

    %create points
    aveR = center(r);
    aveB = center(b);

    %get x and y components
    xComp = aveR(1)-aveB(1);
    yComp = -1*(aveR(2)-aveB(2));

    %get angle
    q2 = xComp < 0 & yComp > 0;
    q3 = xComp < 0 & yComp < 0;
    theta = atan(yComp/xComp);
    if theta < 0 % change from [-pi/2 pi/2] to [0 2pi]
        theta = 2*pi + theta;
    end
    if q2
        theta = theta - pi;
    elseif q3
        theta = theta + pi;
    end
    if i > 1
        pos(numel(pos)+1) = theta;
        xAxis(numel(xAxis)+1) = i+1;
    else
        pos = theta;
        xAxis = 1;
    end
end

pos = unwrap(pos); %fix discontinuity
pos = pos.*(180/pi); %rads --> degrees
pos = smooth(pos,span);
%velo = diff(smooth(pos, span));
velo = diff(pos);
%accel = diff(smooth(velo, span));
accel = diff(velo);
xAxis = xAxis./frameRate;
xAxis2 = xAxis(1:numel(xAxis)-1);
xAxis3 = xAxis(1:numel(xAxis2)-1);

%plot position
subplot(3,1,1);
plot(xAxis, pos, '-b', 'linewidth', 2)
title(header1);
xlim([0 max(xAxis)]);
ylim([0 max(pos)]);
xlabel(xLabel); ylabel(yLabel1);

%plot velocity
subplot(3,1,2);
plot(xAxis2, velo);
title(header2);
xlim([0 max(xAxis)]);
ylim([min(velo) max(velo)]);
xlabel(xLabel); ylabel(yLabel2);

%plot acceleration
subplot(3,1,3);
plot(xAxis3, accel);
title(header3);
xlim([0 max(xAxis)]);
ylim([min(accel) max(accel)]);
xlabel(xLabel); ylabel(yLabel3);

function [aveX, aveY] = center(channel)
    [row,col] = find(channel == 255);
    aveX = [mean(col) mean(row)];
    aveY = [mean(col) mean(row)];
end