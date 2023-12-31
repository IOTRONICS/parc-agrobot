function [BW,maskedRGBImage, firstBrightPixelColumn] = Tyre(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 28-Jul-2023
%------------------------------------------------------


% Convert RGB image to chosen color space
%I = rgb2lab(RGB);
 I = imresize(rgb2lab(RGB),[300,300]);
distance_from_right= 0
firstBrightPixelColumn = 0
% Define thresholds for channel 1 based on histogram settings
channel1Min = 12.131;
channel1Max = 32.858;

% Define thresholds for channel 2 based on histogram settings
channel2Min = -9.532;
channel2Max = 37.909;

% Define thresholds for channel 3 based on histogram settings
channel3Min = -18.510;
channel3Max = 1.959;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
% Perform dilation on the binary mask
    se_dilation = strel('disk', 8); % Adjust the disk size as needed
    BW = imdilate(BW, se_dilation);

    % Perform erosion on the dilated mask
    se_erosion = strel('disk', 2); % Adjust the disk size as needed
    BW = imerode(BW, se_erosion);
% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
% Count the number of white blobs (connected components) in the eroded mask
    labeled_mask = bwlabel(BW);
    
    % Find the properties of connected components
    blob_props = regionprops(labeled_mask, 'Area');

    % Find the index of the largest blob based on its area
    [~, idx] = max([blob_props.Area]);

    % Get the number of pixels for the largest blob
  
   % Assuming you have a binary image named 'binaryImage'

try
   largest_blob_pixels = blob_props(idx).Area;
catch exception
    largest_blob_pixels = 1;
end
 if (largest_blob_pixels > 1)
cc = bwconncomp(BW);

% Get the properties of the connected components
props = regionprops(cc, 'Area', 'PixelIdxList');

% Initialize a binary image to retain only the largest blob
largestBlobImage = false(size(BW));

% Find the largest connected component (blob) based on area
[~, idx] = max([props.Area]);

% Set the pixels of the largest blob to 1 in the new binary image
largestBlobImage(cc.PixelIdxList{idx}) = true;
BW=largestBlobImage;


 end

    if (largest_blob_pixels > 600)
% Assuming you have a binary image named 'binaryImage'

% Find the size of the binary image
[rows, columns] = size(BW);

%disp(columns)

% Initialize the column value of the first bright pixel from the right
firstBrightPixelColumn = -1;
counter = 0

for col = 1:columns
    % Check if there are any white pixels (value 1) in the current column
    if any(BW(:, col) == 1)
        % Store the index of the first column with white pixels
        firstBrightPixelColumn = col;
        %counter = counter +1
        break; % Stop the loop as we found the first bright column
    end
end

%disp(counter)
% Display the result
%if firstBrightPixelColumn ~= -1
%fprintf('First bright pixel column value from tyre: %d\n', firstBrightPixelColumn);
%else
  %  fprintf('No bright pixels found in the image.\n');
%end

    end