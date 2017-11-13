clear all
tic
img_left = imread('F:\\UB CSE\\CSE 573 CVIP\\HWs\\hw3\\data\\part1\\uttower\\left.jpg');
img_right = imread('F:\\UB CSE\\CSE 573 CVIP\\HWs\\hw3\\data\\part1\\uttower\\right.jpg');

%img_left = imread('F:\\UB CSE\\CSE 573 CVIP\\HWs\\hw3\\data\\part1\\ledge\\1.jpg');
%img_right = imread('F:\\UB CSE\\CSE 573 CVIP\\HWs\\hw3\\data\\part1\\ledge\\2.jpg');
color_left = img_left;                          color_right = img_right;
img_left = histeq(im2double(rgb2gray(img_left)), 200);     img_right = histeq(im2double(rgb2gray(img_right)), 200);
[h_l, w_l] = size(img_left);                    [h_r, w_r] = size(img_right);
sigma = 2;      thresh = 0.04;      radius = 2;         display_val = 0;

% Detect image features %
[~, row_left, col_left] = harris(img_left, sigma, thresh, radius, display_val);
[~, row_right, col_right] = harris(img_right, sigma, thresh, radius, display_val);

% Displaying images with detected features %
%{
figure; imshow([color_left color_right]); hold on; 
plot(col_left,row_left,'ys'); plot(col_right + w_l, row_right, 'ys'); 
%}

% Extract local neighborhood for detected features and get descriptors %
% And compute dist2 %
rad = 10;
X = createDescriptors(img_left, row_left, col_left, rad);
C = createDescriptors(img_right, row_right, col_right, rad);
dist_mat = dist2(X, C);

% Find putative matches under specified treshold %
putative_threshold = 5;
[left_matched_indices, right_matched_indices] = find(dist_mat < putative_threshold);

matched_row_left  = row_left(left_matched_indices);     matched_col_left  = col_left(left_matched_indices);
matched_row_right = row_right(right_matched_indices);   matched_col_right = col_right(right_matched_indices);
[matchCount, ~]= size(left_matched_indices);
matched_left_img = [matched_col_left, matched_row_left, ones(matchCount, 1)];
matched_right_img = [matched_col_right, matched_row_right, ones(matchCount, 1)];

% Displaying image with matched features that pass the threshold %
%{
figure; imshow([color_left color_right]); hold on; title('Top matched features'); hold on; 
plot(matched_col_left, matched_row_left, 'ys'); plot(matched_col_right + w_l, matched_row_right, 'ys');
%}
% Display lines of matched features %
%{
draw_row = [matched_row_left, matched_row_right];
draw_col = [matched_col_left, matched_col_right+w_l];
figure; imshow([color_left color_right]); hold on; title('Lines for top matched features');
hold on;
plot(matched_col_left, matched_row_left, 'ys'); plot(matched_col_right + w_l, matched_row_right, 'ys');
for i = 1:matchCount 
    plot(draw_col(i,:), draw_row(i,:));
end
%}

% RANSAC %
% These two matrices contain the indices of the points we matched that are
% under the threshold specified.  We will use these indices for further operations since these are the
% "putative points"
% Each of the below matrices contain two columns. Each row represents a
% (x,y) pair - that is, a putative point
%modified below matrices to be homogeneous
matchIndicesLeft = [matched_col_left, matched_row_left, ones(matchCount, 1)];
matchIndicesRight = [matched_col_right, matched_row_right, ones(matchCount, 1)];

numIter = 100;
sampleSize = 4;
max_num_inliers = 0;
max_homo = [];
min_ssd = intmax;
count = 0;
inlier_thresh = 300;

homoFeederMat = [];     % we will take SVD of this
for i=1:numIter
    i;
    p = randperm(matchCount, sampleSize);
    %for loop to store random points from left and right images
    for j = 1:sampleSize
        pointsForHomoLeft(j,:) = matchIndicesLeft(p(j),:);
        pointsForHomoRight(j,:) = matchIndicesRight(p(j),:);
        xi = pointsForHomoLeft(j,1);               yi = pointsForHomoLeft(j,2);
        xi_prime = pointsForHomoRight(j,1);        yi_prime = pointsForHomoRight(j,2);        
        p_i = [-xi, -yi, -1,0,0,0, xi*xi_prime, yi*xi_prime, xi_prime;
                0,0,0, -xi, -yi, -1, xi*yi_prime, yi*yi_prime, yi_prime];
        homoFeederMat = [homoFeederMat; p_i];
    end
    
    [U, S, V] = svd(homoFeederMat);
    h = V (:, 9);
    homography = reshape(h, 3, 3);
    x = homography(3,3);
    homography = homography ./ x;
    
    % other way to find homography %
    %homography = pointsForHomoLeft\pointsForHomoRight;
    
    %check = pointsForHomoLeft * homography;
    %scale_down1 = check(:,3);
    %check = check ./ scale_down1;
    %scale_down2 = pointsForHomoRight(:,3);    
    %B = pointsForHomoRight ./ scale_down2;
    
    check = matched_left_img * homography;
    B = matched_right_img;
    dif = finddif (check, B)';
    ssd = (sum(dif(:).^2));    
    num_inliers = nnz(dif < inlier_thresh);
    ssd_store(i) = ssd;
   
    if(num_inliers >= max_num_inliers && i ~= 1 && ssd < min_ssd)
           min_ssd = ssd;           max_num_inliers = num_inliers;           max_homo = homography;
           left_homo_pts = matched_left_img((dif<inlier_thresh), 1:2);      
           right_homo_pts = matched_right_img((dif<inlier_thresh), 1:2);
    end
end
clear num_inliers;
H = max_homo;
mean(sqrt(sqrt(ssd)))

% Display lines of inlier features %
%{
draw_row = [left_homo_pts(:,1), right_homo_pts(:,1)];
draw_col = [left_homo_pts(:,2), right_homo_pts(:,2)+w_l];
figure; imshow([color_left color_right]); hold on; title('Inliers');
hold on;
plot(left_homo_pts(:,1), left_homo_pts(:,2), 'ys'); plot(right_homo_pts(:,1) + w_l, right_homo_pts(:,2), 'ys');
for i = 1:matchCount 
    %plot(draw_col(i,:), draw_row(i,:));
end
%}
% Displaying images with inlier features %
%{
col_l = left_homo_pts(:,2);
row_l = left_homo_pts(:,1);
col_r = right_homo_pts(:,2);
row_r = right_homo_pts(:,1);
figure, imshow([color_left color_right]); hold on; 
plot(col_l,row_l,'ys'); plot(col_r + w_l, row_r, 'ys'); 
%}

homo_trans = maketform('projective', H);
[left_trans, xdata, ydata] = imtransform(color_left, homo_trans);

%figure, imshow(left_trans);
firstcol = xdata(1);    lastcol = xdata(2);     firstrow = ydata(1);    lastrow = ydata(2);

%dist_mov = (w_l - left_homo_pts(1,1)) +  right_homo_pts(1,1);
%xoffset = w_l - dist_mov;

xsize = w_r+xoffset;
ysize = lastrow - firstrow;
comp = zeros(ceil(ysize), ceil(xsize));
%comp(xoffset, ysize-h_r) = color_right;
%figure, imshow(comp)

toc