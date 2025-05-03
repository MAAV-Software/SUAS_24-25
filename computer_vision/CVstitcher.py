import numpy as np
import os
import re
import cv2
import time

def fit_homography(pts1 ,pts2):
    """
    Given a set of N correspondences of the form [x,y], [x',y'],
    fit a homography that maps [x',y',1] to [x,y,1] using the analytic solution.

    Input:
        pts1, pts2: (N, 2) matrices representing N corresponding points [x, y] and [x', y']
    Returns:
        H: a (3,3) homography matrix that satisfies [x,y,1] == H [x',y',1]
    """
    H = None

    # Get x and y coordinates for the destination and the reference points
    x = pts1[:, 0]
    y = pts1[:, 1]

    x_p = pts2[:, 0]
    y_p = pts2[:, 1]

    A = []
    # Define the analytic solution matrix
    for i in range(pts1.shape[0]):
        A.append([x[i], y[i], 1, 0, 0, 0, -x[i] * x_p[i], -y[i] * x_p[i], -x_p[i]])
        A.append([0, 0, 0, x[i], y[i], 1, -x[i] * y_p[i], -y[i] * y_p[i], -y_p[i]])
    A = np.array(A)

    # Apply the SVD
    U, S, Vt = np.linalg.svd(A)
    h = Vt[-1, :]
    H = h.reshape(3, 3)

    # Normalize to make the bottom right value 1
    H = H / H[2, 2]

    return H

def to_homog(pts):
    """
    Convert 2D catesian coordinates to homogenious coordinates
    Input:
        pts: (N, 2) numpy array
    Returns:
        new_pts: (N, 3) numpy array
    """

    new_pts = None

    # Check if pts is a single point (1D) or a set of points (2D)
    if pts.ndim == 1:
        # If it's a single point, reshape it to a 2D array with one row
        pts = pts.reshape(1, -1)

    # Stack a layer of ones to the end of the points so that it has (x, y, 1)
    new_pts = np.hstack((pts, np.ones((pts.shape[0],1))))

    return new_pts


def to_cart(pts):
    """
    Convert 2D homogenious coordinates to cartesian coordinates
    Input:
        pts: (N, 3) numpy array
    Returns:
        new_pts: (N, 2) numpy array
    """

    new_pts = None

    # Convert back to the original by getting rid of the final value and dividing by it
    cartesian_points = pts[:, 0:2]
    normalization_factor = pts[:, 2]
    normalization_factor = normalization_factor[:, np.newaxis]

    new_pts = cartesian_points / normalization_factor

    return new_pts

def get_sift_features(img):
    """
    Compute SIFT features using cv2 library functions.
    Use default parameters when computing the keypoints.

    Input:
        img: cv2 image
    Returns:
        keypoints: a list of cv2 keypoints
        descriptors: a list of SIFT descriptors
    """

    keypoints = None
    descriptors = None

    # Use opencv2's sift detector to calculate keypoints and descriptors through SIFT
    sift = cv2.SIFT_create()
    keypoints, descriptors = sift.detectAndCompute(img,None)

    return keypoints, descriptors


def match_keypoints(desc_1, desc_2, ratio=0.75):
    """
    Compute matches between feature descriptors of two images using Lowe's ratio test.

    Input:
        desc_1, desc_2: list of feature descriptors
        ratio (optional): Lowe's ratio test threshold
    Return:
        matches: list of feature matches
    """

    matches = []

    # Use cv2's descriptro matcher to get 2 potential matches for each descriptor. Apply Lowe's ratio to see which are usable
    matcher = cv2.DescriptorMatcher_create("BruteForce")
    knn_matches = matcher.knnMatch(desc_1, desc_2, 2)

    for first, second in knn_matches:
      # If the first.distance < ratio * second.distance, it means that they are far apart enough and first is clearly the best feature
      if first.distance < ratio * second.distance:
        matches.append(first)

    return matches


def transform_ransac(x1, x2, verbose=False):
    """
    Implements RANSAC to estimate homography matrix.

    Input:
        x1, x2: (N, 2) matrices
    Return:
        best_model: homography matrix with most inliers
    """

    best_model = None

    most_inliers_so_far = 0
    Homos = None

    # Run interations to calculate the number of inliers for randomly assigned points
    for iteration in range(1000):
        indices = np.random.choice(x1.shape[0], 4, replace=False)
        
        H = fit_homography(x1[indices], x2[indices])

        # Convert points to homogeneous coordinates
        x1_homog = to_homog(x1)  # Shape: (N, 3)
        x2_homog = to_homog(x2)  # Shape: (N, 3)
        
        # Apply homography to all points at once (matrix multiplication)
        homog_dest = (H @ x1_homog.T).T  # Shape: (N, 3)
        
        # Convert back to Cartesian coordinates
        dest = to_cart(homog_dest)  # Shape: (N, 2)
        
        # Calculate Euclidean distance between corresponding points
        distances = np.linalg.norm(dest - x2, axis=1)  # Shape: (N,)

        # Determine inliers based on distance threshold
        inliers = distances < 2.0

        # Count inliers
        num_inliers = np.sum(inliers)

        # If we have more inliers than before, update the best model
        if num_inliers > most_inliers_so_far:
            most_inliers_so_far = num_inliers
            best_model = H
            Homos = (H, inliers)

    # Extract the best inliers
    H, inliers = Homos

    # Select the inlier points for both x1 and x2
    pts_1 = x1[inliers]
    pts_2 = x2[inliers]

    # Recompute the homography using the inlier points
    best_model = fit_homography(pts_1, pts_2)

    return best_model


def homography_from_image_pairs(img1, img2):
    """
    Given a pair of overlapping images, generate the homography matrix H.

    Input:
        img1, img2: cv2 images
    Return:
        H: computed homography matrix
    """

    H = None

    # Get the keypoints and descriptors for each image
    kp1, desc1 = get_sift_features(img1)
    kp2, desc2 = get_sift_features(img2)

    # calculate the potential matches given the two descriptors
    matches = match_keypoints(desc1, desc2)

    pts_1 = np.float32([kp1[m.queryIdx].pt for m in matches])
    pts_2 = np.float32([kp2[m.trainIdx].pt for m in matches])

    # Find the proper homography matrix given the matched up points
    H = transform_ransac(pts_2, pts_1)

    return H


def panoramic_stitching(*imgs):
    """
    Given a list of overlapping images, generate a panoramic image.

    Input:
        imgs: cv2 images
    Return:
        final_img: cv2 image of panorama
    """

    final_img = None

    first_img = imgs[0]
    height_base_image, width_base_image, channels = first_img.shape

    # Create and initialize a blank canvas that we will be stitching on
    final_img_width = width_base_image * len(imgs)
    final_img_height = height_base_image * len(imgs)
    final_img = np.zeros((final_img_height, final_img_width, 3), dtype=np.uint8)

    # Place the first image on the left hand side but in the middle of the y-axis so it's not in a corner or anything, this will prevent cropping
    middle_y_position = (final_img_height - height_base_image) // 2
    final_img[middle_y_position:middle_y_position + height_base_image, 0:width_base_image] = first_img
    
    # First homography will be just the identify matrix, does nothing to the photo
    applied_H = np.eye(3)

    # Loop through all the images
    for i, img in enumerate(imgs):
        # For images that are not the first, since we do not need to stitch the first, it is already placed onto the canvas
        if i > 0:
            prev_img = imgs[i - 1]

            if i == 1:
              H = homography_from_image_pairs(final_img, img) # This is saying that for the first image, we want to map to the large canvas, as that is where we will be stitching
            else:
              H = homography_from_image_pairs(prev_img, img) # For subsequent images, we just need to figure out how to stitch them together without worring about the canvas too much (it's already taken care of)

            applied_H = applied_H @ H
            
            # Get the warped images with the updated homography matrix
            warped_img = cv2.warpPerspective(img, applied_H, (final_img_width, final_img_height))

            warped_height, _, __ = warped_img.shape

            # Stich them together, making sure to stitch where the canvas isn't black
            mask = warped_img > 0
            final_img[mask] = warped_img[mask]

    # Remove all black rows (rows where all channels are 0)
    non_black_rows = np.any(final_img != 0, axis=(1, 2))  # Check if any pixel in the row is non-black (non-zero)
    final_img = final_img[non_black_rows]

    # Remove all black columns (columns where all channels are 0)
    non_black_columns = np.any(final_img != 0, axis=(0, 2))  # Check if any pixel in the column is non-black (non-zero)
    final_img = final_img[:, non_black_columns]

    return final_img


def extract_number(filename):
    match = re.search(r'\d+', filename)
    return int(match.group()) if match else -1

def load_images_from_folder(directory, downsample):
    images = []
    # Load the folder and read in the images and put them in a list
    files = [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]

    files.sort(key=lambda f: extract_number(os.path.basename(f)))

    for file in files:
        print(file)

    for file in files:
        if '.DS_Store' not in file:
            img = cv2.imread(file)
            if img is not None:
                if downsample:
                    img = cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
                    images.append(img)
                else:
                    images.append(img)
            
    return images

image_folder = input("Enter filename here: ") # Get the folder of images
downsample = input("Do you want to downsample: ") # Ask the user if they want to downsample (leads to quicker runtime, but worse resolution)
if downsample == "Yes" or downsample == "yes" or downsample == "y" or downsample == "Y":
    downsample = True
else:
    downsample = False
start_time = time.perf_counter()
list_of_images = load_images_from_folder(image_folder, downsample) # Load images from the folder
print("Num of images to be stitched", len(list_of_images))
result = panoramic_stitching(*list_of_images) # Stitch the images together like in a panorama
cv2.imwrite('result.jpg', result) # Save the final image

end_time = time.perf_counter()
runtime = end_time - start_time
print(f"Runtime: {runtime:.4f} seconds") # Print out the runtime
