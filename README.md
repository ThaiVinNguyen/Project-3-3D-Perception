[//]: # (Image References)
[pr2_robot]: ./misc_image/pr2_robot.png
[voxel_grid_filter]: ./misc_image/voxel_filter.png
[passthrough_filter]: ./misc_image/passthrough_filter.png
[extracted_inliers]: ./misc_image/extracted_inliers.png
[extracted_outliers]: ./misc_image/extracted_outliers.png
[object_cluster_cloud]: ./misc_image/object_cluster_cloud.png
[confusion_matrices]: ./misc_image/accuracy_pr2.png
[test_1_object_recognition]: ./misc_image/test1_recognition.png
[test_2_object_recognition]: ./misc_image/test2_recognition.png
[test_3_object_recognition]: ./misc_image/test3_recognition.png

# 3D Perception Project

![pr2 robot][pr2_robot]

This project focuses on 3D perception using a PR2 robot simulation utilizing an RGB-D camera. The goal of perception is to convert sensor input into a point cloud image where specific objects can be identified and isolated.

The three main parts of perception include filtering the point cloud, clustering relevant objects, and recognizing objects.

## Contents

- [Filtering](#filtering)
- [Clustering For Segmentation](#clustering-for-segmentation)
- [Capture And Train Model Object Recognition](#capture-and-train-model-object-recognition)
- [Pr2 Arm Robot Working](#pr2-arm-robot-working)
- [Pr2 Robot Simulation And Regconition](#pr2-robot-simulation-and-recognition)
- [Code Functions](#code-functions)
- [Result](#result)
- [Improvement and Development](#improvement-and-development)


## Filtering

### Voxel Grid Filter

![voxel grid filter][voxel_grid_filter]

The raw point cloud will often have more details than required, causing excess processing power use when analyzing it.

A voxel grid filter downsamples the data by taking a spatial average of the points in the cloud confined by each voxel. The set of points which lie within the bounds of a voxel are assigned to that voxel and are statistically combined into one output point.

I used an X, Y, and Z voxel grid filter leaf size of *0.01*. This was a good compromise of leaving enough detail while minimizing processing time.

### Passthrough Filter

![passthrough filter][passthrough_filter]

The passthrough filter allows a 3D point cloud to be cropped by specifying an axis with cut-off values along that axis. The region allowed to *pass through* is often called the *region of interest*.

The PR2 robot simulation required passthrough filters for both the Y and Z axis (global). This prevented processing values outside the area immediately in front of the robot. For the Y axis, I used a range of *-0.45* to *0.45*, and for the Z axis, I used a range of *0.6* to *1.0*.

### RANSAC Plane Segmentation

Random Sample Consensus (RANSAC) is used to identify points in the dataset that belong to a particular model. It assumes that all of the data in a dataset is composed of both inliers and outliers, where inliers can be defined by a particular model with a specific set of parameters, and outliers don't.

I used a RANSAC max distance value of *0.01*.

The extracted inliers includes the table. It looks like this:

![RANSAC plane segmentation - extracted inliers][extracted_inliers]

The extracted outliers contains the objects on the table, and looks like this:

![RANSAC plane segmentation - extracted outliers][extracted_outliers]

## Clustering For Segmentation

With a cleaned point cloud, it is possible to identify individual objects within it. One straightforward approach is to use Euclidean clustering to perform this calculation.

The two main algorithms possible include:

- K-means

- DBSCAN

### K-means

K-means clustering algorithm is able to group data points into n groups based on their distance to randomly chosen centroids. However, K-means clustering requires that you know the number of groups to be clustered, which may not always be the case.

### DBSCAN

Density-based spatial cluster of applications with noise (DBSCAN) (sometimes called *Euclidean clustering*) is a clustering algorithm that creates clusters by grouping data points that are within some threshold distance from their nearest neighbor.

DBSCAN is unique over k-means because you don't need to know how many clusters to expect in the data. However, you do need to know something about the density of the data points being clustered.

Performing a DBSCAN within the PR2 simulation required converting the XYZRGB point cloud to a XYZ point cloud, making a k-d tree (decreases the computation required), preparing the Euclidean clustering function, and extracting the clusters from the cloud. This process generates a list of points for each detected object.

By assigning random colors to the isolated objects within the scene, I was able to generate this cloud of objects:

![object cluster cloud][object_cluster_cloud]

## Capture And Train Model Object Recognition

The object recognition code allows each object within the object cluster to be identified. In order to do this, the system first needs to train a model to learn what each object looks like. Once it has this model, the system will be able to make predictions as to which object it sees.

### Capture Object Features

Color histograms are used to measure how each object looks when captured as an image. Each object is positioned in random orientations to give the model a more complete understanding. For better feature extraction, the RGB image can be converted to HSV before being analyzed as a histogram. The number of bins used for the histogram changes how detailed each object is mapped, however too many bins will over-fit the object.

The code for building the histograms can be found in [features.py](pr2_robot_project/src/sensor_stick/src/sensor_stick/features.py).

The [capture_features_pr2_robot.py](pr2_robot_project/src/sensor_stick/scripts/capture_features_pr2_robot.py) script saved the object features to a file named `training_set_pr2_robot.sav`. It captured each object in *50* random orientations, using the *HSV* color space and *128* bins when creating the image histograms.

### Train SVM Model

A support vector machine (SVM) is used to train the model (specifically a SVC). The SVM loads the training set generated from the `capture_features_pr2_robot.py` script, and prepares the raw data for classification.

I was able to generate an accuracy score of 95.32%. The [train_svm.py](pr2_robot_project/src/RoboND_Perception_Project/pr2_robot/scripts/train_svm.py) script trained the SVM, saving the final model as `model_pr2_robot.sav`.

The confusion matrices below shows the non-normalized and normalized results for a test case using the trained model generated above.

![Confusion matrices for the non-normalized and normalized test case][confusion_matrices]

##Pr2 Arm Robot Working

- Run simulator:
    *roslaunch pr2_robot pick_place_project.launch*

- Object Recognition:
     *rosrun pr2_robot recognition_project.py*

## Pr2 Robot Simulation And Recognition

The PR2 robot simulation has three test scenarios to evaluate the object recognition performance. The following sections demonstrate each scenario.

### Test 1

![Test 1 object recognition][test_1_object_recognition]

View the goal [pick list](pr2_robot_project/src/RoboND_Perception_Project/pr2_robot/config/pick_list_1.yaml), and the calculated [output values](pr2_robot_project/output_1.yaml).

### Test 2

![Test 2 object recognition][test_2_object_recognition]

View the goal [pick list](pr2_robot_project/src/RoboND_Perception_Project/pr2_robot/config/pick_list_2.yaml), and the calculated [output values](pr2_robot_project/output_2.yaml).

### Test 3

![Test 3 object recognition][test_3_object_recognition]

View the goal [pick list](pr2_robot_project/src/RoboND_Perception_Project/pr2_robot/config/pick_list_3.yaml), and the calculated [output values](pr2_robot_project/output_3.yaml).

## Code Functions

Perception exercise code can be found [here](RoboND_Perception_Exercises).
The PR2 3D perception project code can be found [here](pr2_robot_project/src).

## Result
- Completed exercise 1.
- Completed exercise 2.
- Completed exercise 1.
- Completed 3D Pr2 robot arm project. But I only achived and need to improve in the future.
		100% (3/3) objects in test1.world
		100% (5/5) objects in test2.world
		75% (6/8) objects in test3.world

## Improvement And Development

- Improve parameters and functions for can achive 100% (8/8) objects in test3.world.
- Add code for pr2 robot can active. 