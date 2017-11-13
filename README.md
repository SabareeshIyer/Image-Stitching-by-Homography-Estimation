# Image-Stitching-by-Homography-Estimation
Stitching pairs of images

1. Load both images, convert to double and to grayscale.
2. Detect feature points in both images using the Harris corner detector.
3. Extract local neighborhoods around every keypoint in both images, and form descriptors simply by "flattening" the pixel values in each neighborhood to one-dimensional vectors.
4. Compute distances between every descriptor in one image and every descriptor in the other image using dist2.
5. Select putative matches based on the matrix of pairwise descriptor distances obtained above. Select all pairs whose descriptor distances are below a specified threshold.
6. Run RANSAC to estimate a homography mapping one image onto the other.
7. Warp one image onto the other using the estimated transformation.
8. Combine images.

## RANSAC
Number of iterations required vary from image to image. More "difficult" images require a higher number of iterations and better features to be extracted to get a proper homography and achieve optimal warping and transformation.

### Terms of interest
- RANSAC
- Homography
- SVD
- Corner detection
- Warping
- Stitching
