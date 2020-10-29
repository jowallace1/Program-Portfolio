%setup
clc
clear all

%user defined variables
fileName = 'test.mp4';
span = 0.1;
samplingSize = 1;
maxFrame = 100;
frameMax = 0; %set to 1 for maxFrame to be equivalent to numFrames

%do not change
addpath('Videos');
folder = fileparts(which(fileName));
xAxis = 1;
header1 = 'Position vs Frame Count';
xLabel = 'Frame Count';
yLabel1 = 'Position (Degrees)';
header2 = 'Velocity vs Frame Count';
yLabel2 = 'Angular Velocity (Degrees/Frame)';
header3 = 'Acceleration vs Frame Count';
yLabel3 = 'Angular Acceleration';
v = VideoWriter('masked');

%get video
if ~exist(fileName, 'file')
    disp("The file " + fileName + " does not exist in this folder. Please try again")
else
    disp("The file " + fileName + " has been successfully targeted.")   
end
videoObject = VideoReader(fileName); %create video object
numFrames = videoObject.numFrames; %get number of frames

if frameMax == 1
    maxFrame = numFrames;
end

open(v);
for i = 1:samplingSize:maxFrame
    frame = read(videoObject, i);
    %get channels
    r = frame(:,:,1);
    g = frame(:,:,2);
    b = frame(:,:,3);

    %create masks
    %highTHoldR = 150; %use to create unique color thresholds
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
    
    mask = cat(3,r,g,b);
    aveR = center(r);
    aveB = center(b);
    mask(1,1,1) = 255;
    mask(1,1,1) = 255;
    writeVideo(v,mask);
end
close(v);

function [aveX, aveY] = center(channel)
    [row,col] = find(channel == 255);
    aveX = [mean(col) mean(row)];
    aveY = [mean(col) mean(row)];
end