%% function GreencolourDetection() %% Colour segmentation

clc;	
clear;	
close all;	
%% read image

[rgbImage, storedColorMap] = imread('img6.jpg'); %onion.png
[rows, columns, numberOfColorBands] = size(rgbImage); 
figure,imshow(rgbImage); title('Input Image');
%% Convert RGB image to HSV

hsvImage = rgb2hsv(rgbImage);
figure,imshow(hsvImage);title('HSV Image');
figure();
hImage = hsvImage(:,:,1);
subplot(2,3,1);imshow(hImage);title('hImage');
sImage = hsvImage(:,:,2);
subplot(2,3,2);imshow(sImage);title('sImage');
vImage = hsvImage(:,:,3);
subplot(2,3,3);imshow(vImage);title('vImage');
%% Compute the histogram of the 3 bands

[hueCounts, hueBinValues] = imhist(hImage); 
maxHueBinValue = find(hueCounts > 0, 1, 'last'); 
maxCountHue = max(hueCounts);
hHuePlot = subplot(2, 3, 4);
area(hueBinValues, hueCounts, 'FaceColor', 'g'); 
grid on; 
xlabel('Hue Value'); 
ylabel('Pixel Count'); 
title('Histogram of Hue Image');

[saturationCounts, saturationBinValues] = imhist(sImage); 
maxSaturationBinValue = find(saturationCounts > 0,1,'last'); 
% to find the number of bins
maxCountSaturation = max(saturationCounts); 
% to find the maximum value
SaturationPlot = subplot(2, 3, 5);
area(saturationBinValues, saturationCounts, 'FaceColor', 'g'); 
grid on; 
xlabel('Saturation Value'); 
ylabel('Pixel Count'); 
title('Saturation');

[valueCounts, valueBinValues] = imhist(vImage); 
maxValueBinValue = find(valueCounts > 0, 1, 'last'); 
maxCountValue = max(valueCounts);
ValuePlot = subplot(2, 3, 6);
area(valueBinValues, valueCounts, 'FaceColor', 'b'); 
grid on; 
xlabel('Value Value'); 
ylabel('Pixel Count'); 
title('Value');

%% setting threshold for Green colour
hueThresholdLow = 0.15;
hueThresholdHigh = 0.60;
saturationThresholdLow = 0.36;
saturationThresholdHigh = 1;
valueThresholdLow = 0;
valueThresholdHigh = 0.8;

%% Green Colour detection

% apply each color band's particular thresholds to the color band
hueMask = (hImage >= hueThresholdLow) & (hImage <= hueThresholdHigh);
saturationMask = (sImage >= saturationThresholdLow) & (sImage <= saturationThresholdHigh);
valueMask = (vImage >= valueThresholdLow) & (vImage <= valueThresholdHigh);

figure();
subplot(1, 3, 1);
imshow(hueMask, []);
title('Hue Mask');
subplot(1, 3, 2);
imshow(saturationMask, []);
title('Saturation Mask');
subplot(1, 3, 3);
imshow(valueMask, []);
title('Value Mask');

% Set all axes to be the same width and height.
% This makes it easier to compare them.
maxCount = max([maxCountHue,  maxCountSaturation, maxCountValue]); 
axis([hHuePlot SaturationPlot ValuePlot], [0 1 0 maxCount]); 


% Plot all 3 histograms in one plot.
figure(); 
plot(hueBinValues, hueCounts, 'r', 'LineWidth', 2); 
grid on; 
xlabel('Values'); 
ylabel('Pixel Count'); 
hold on; 
plot(saturationBinValues, saturationCounts, 'g', 'LineWidth', 2); 
plot(valueBinValues, valueCounts, 'b', 'LineWidth', 2); 
title('Histogram of All Bands'); 
maxGrayLevel = max([maxHueBinValue, maxSaturationBinValue, maxValueBinValue]); % Just for our information....
% Make x-axis to just the max gray level on the bright end. 
xlim([0 1])

% Combine the masks to find where all 3 are "true"
coloredObjectsMask = uint8(hueMask & saturationMask & valueMask);
figure,imshow(coloredObjectsMask,[]);

%smallestAcceptableArea = 10; % Keep areas only if they're bigger than this
%coloredObjectsMask = uint8(bwareaopen(coloredObjectsMask, smallestAcceptableArea));

structuringElement = strel('disk', 2);
coloredObjectsMask = imclose(coloredObjectsMask, structuringElement);
figure,imshow(coloredObjectsMask,[]);
coloredObjectsMask = imfill(logical(coloredObjectsMask), 'holes');
figure,imshow(coloredObjectsMask,[]);
coloredObjectsMask = cast(coloredObjectsMask, 'like', rgbImage); 

maskedImageR = coloredObjectsMask .* rgbImage(:,:,1);
maskedImageG = coloredObjectsMask .* rgbImage(:,:,2);
maskedImageB = coloredObjectsMask .* rgbImage(:,:,3);

maskedRGBImage = cat(3, maskedImageR, maskedImageG, maskedImageB);
figure,imshow(maskedRGBImage); 
    
   

	
