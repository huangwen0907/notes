# Feature Detection and Description
##  Harris Corner Detection
In last chapter, we saw that corners are regions in the image with large variation in intensity in all the directions. One early attempt to find these corners was done by **Chris Harris & Mike Stephens** in their paper **A Combined Corner and Edge Detector** in 1988, so now it is called Harris Corner Detector. He took this simple idea to a mathematical form. It basically finds the difference in intensity for a displacement of $[u,v]$ in all directions. This is expressed as below:
$$
E(u,v) = \sum_{x,y} \underbrace{w(x,y)}_{\text{window function}} \underbrace{[I(x+u,y+v)}_{\text{shifted intensity}} - \underbrace{I(x,y)]^2}_{\text{intensity}} 
$$
Window function is either a rectangular window or gaussian window which gives weights to pixels underneath.

We have to maximize this function $E(u,v)$ for corner detection.
That means, we have to maximize the second term. Applying Taylor Expansion to above equation and using some mathematical steps (please refer any standard text books you like for full derivation), we get the final equation as:
$$
E(u,v) \approx \begin{bmatrix} u & v \end{bmatrix} M \begin{bmatrix} u \\ v \end{bmatrix}
$$
where:
$$
M = \sum_{x,y} w(x,y) \begin{bmatrix} I_x I_x  & I_x I_y \\ I_x I_y & I_y I_y\end{bmatrix}
$$ 
here $I_x and I_y$   are image derivatives in x and y directions respectively.
Then comes the main part. After this, they created a score, basically an equation, which will determine if a window can contain a corner or not.
$$
R = det(M) -k(trace(M))^2
$$

where 
$$
det(M)  = \lambda_1 \lambda_2 \\
 trace(M) = \lambda_1 +\lambda_2 \\
 \lambda_1\ and\ \lambda_2\ are\ the\ eigen\ values\  of\ M
 $$ 
 
So the values of these eigen values decide whether a region is corner, edge or flat.

- When$|R|$ is small, which happens when $\lambda_1$ and $\lambda_2$ are small, the region is flat.
-   When  $R < 0$, which happens when $\lambda_1 >> \lambda_2$ or vice versa, the region is edge.
-   When  $R$  is large, which happens when  $\lambda_1$  and $\lambda_2$   are large and  $\lambda_2 \sim \lambda_2$, the region is a corner.
It can be represented in a nice picture as follows:
![](https://docs.opencv.org/3.0-beta/_images/harris_region.jpg)
So the result of Harris Corner Detection is a grayscale image with these scores. Thresholding for a suitable give you the corners in the image. We will do it with a simple image.

## # Shi-Tomasi Corner Detector & Good Features to Track
**Goal**
In this chapter,
- We will learn about the another corner detector: Shi-Tomasi Corner Detector
- We will see the function:  **cv2.goodFeaturesToTrack()**

**Theory**
In last chapter, we saw Harris Corner Detector. Later in 1994, J. Shi and C. Tomasi made a small modification to it in their paper **Good Features to Track** which shows better results compared to Harris Corner Detector. The scoring function in Harris Corner Detector was given by:
$$
R = \lambda_1 \lambda_2 - k(\lambda_1+\lambda_2)^2
$$
Instead of this, Shi-Tomasi proposed:
$$
R =min(\lambda_1,\lambda_2)
$$
If it is a greater than a threshold value, it is considered as a corner. If we plot it in $\lambda_1 - \lambda_2$ space as we did in Harris Corner Detector, we get an image as below:
![shitomasi_space](https://docs.opencv.org/3.0-beta/_images/shitomasi_space.png)

From the figure, you can see that only when $\lambda_1$ and $\lambda_2$ are above a minimum value, $\lambda_{min}$, it is conidered as a corner(green region).

## Introduction to SIFT (Scale-Invariant Feature Transform)
**Goal**
In this chapter,
-   We will learn about the concepts of SIFT algorithm
-   We will learn to find SIFT Keypoints and Descriptors.

**Theory**
In last couple of chapters, we saw some corner detectors like Harris etc. They are rotation-invariant, which means, even if the image is rotated, we can find the same corners. It is obvious because corners remain corners in rotated image also. But what about scaling? A corner may not be a corner if the image is scaled. For example, check a simple image below. A corner in a small image within a small window is flat when it is zoomed in the same window. So Harris corner is not scale invariant.
![sift_scale_invariant](https://docs.opencv.org/3.0-beta/_images/sift_scale_invariant.jpg)

So, in 2004,  **D.Lowe**, University of British Columbia, came up with a new algorithm, Scale Invariant Feature Transform (SIFT) in his paper,  **Distinctive Image Features from Scale-Invariant Keypoints**, which extract keypoints and compute its descriptors.  _(This paper is easy to understand and considered to be best material available on SIFT. So this explanation is just a short summary of this paper)_.
There are mainly four steps involved in SIFT algorithm. We will see them one-by-one.

### 1. Scale-space Extrema Detection
---
From the image above, it is obvious that we can’t use the same window to detect keypoints with different scale. It is OK with small corner. But to detect larger corners we need larger windows. For this, scale-space filtering is used. In it, Laplacian of Gaussian is found for the image with various $\sigma$ values. LoG acts as a blob detector which detects blobs in various sizes due to change in $\sigma$. In short, $\sigma$ acts as a scaling parameter. For eg, in the above image, gaussian kernel with low $\sigma$ gives high value for small corner while guassian kernel with high $\sigma$ fits well for larger corner. So, we can find the local maxima across the scale and space which gives us a list of $(x,y,\sigma)$ values which means there is a potential keypoint at $(x,y)$ at $\sigma$ scale.

But this LoG is a little costly, so SIFT algorithm uses Difference of Gaussians which is an approximation of LoG. Difference of Gaussian is obtained as the difference of Gaussian blurring of an image with two different $\sigma$, let it be $\sigma$ and $k\sigma$. This process is done for different octaves of the image in Gaussian Pyramid. It is represented in below image:

![sift_dog](https://docs.opencv.org/3.0-beta/_images/sift_dog.jpg)

Once this DoG are found, images are searched for local extrema over scale and space. For eg, one pixel in an image is compared with its 8 neighbours as well as 9 pixels in next scale and 9 pixels in previous scales. If it is a local extrema, it is a potential keypoint. It basically means that keypoint is best represented in that scale. It is shown in below image:

![](https://docs.opencv.org/3.0-beta/_images/sift_local_extrema.jpg)

Regarding different parameters, the paper gives some empirical data which can be summarized as, number of octaves = 4, number of scale levels = 5, initial $\sigma = 1.6$, $k=\sqrt{2}$ etc as optimal values.
### 2. Keypoint Localization
---
Once potential keypoints locations are found, they have to be refined to get more accurate results. They used Taylor series expansion of scale space to get more accurate location of extrema, and if the intensity at this extrema is less than a threshold value (0.03 as per the paper), it is rejected. This threshold is called  **contrastThreshold**  in OpenCV

DoG has higher response for edges, so edges also need to be removed. For this, a concept similar to Harris corner detector is used. They used a 2x2 Hessian matrix (H) to compute the pricipal curvature. We know from Harris corner detector that for edges, one eigen value is larger than the other. So here they used a simple function,

If this ratio is greater than a threshold, called  **edgeThreshold**  in OpenCV, that keypoint is discarded. It is given as 10 in paper.

So it eliminates any low-contrast keypoints and edge keypoints and what remains is strong interest points.

### 3. Orientation Assignment
---
Now an orientation is assigned to each keypoint to achieve invariance to image rotation. A neigbourhood is taken around the keypoint location depending on the scale, and the gradient magnitude and direction is calculated in that region. An orientation histogram with 36 bins covering 360 degrees is created. (It is weighted by gradient magnitude and gaussian-weighted circular window with  $\sigma$  equal to 1.5 times the scale of keypoint. The highest peak in the histogram is taken and any peak above 80% of it is also considered to calculate the orientation. It creates keypoints with same location and scale, but different directions. It contribute to stability of matching.

### 5. Keypoint Matching
---
Keypoints between two images are matched by identifying their nearest neighbours. But in some cases, the second closest-match may be very near to the first. It may happen due to noise or some other reasons. In that case, ratio of closest-distance to second-closest distance is taken. If it is greater than 0.8, they are rejected. It eliminaters around 90% of false matches while discards only 5% correct matches, as per the paper.

So this is a summary of SIFT algorithm. For more details and understanding, reading the original paper is highly recommended. Remember one thing, this algorithm is patented. So this algorithm is included in Non-free module in OpenCV.

##  Introduction to SURF (Speeded-Up Robust Features)
**Goal**
In this chapter,
-   We will see the basics of SURF
-   We will see SURF functionalities in OpenCV

**Theroy**
In last chapter, we saw SIFT for keypoint detection and description. But it was comparatively slow and people needed more speeded-up version. In 2006, three people, Bay, H., Tuytelaars, T. and Van Gool, L, published another paper, **SURF: Speeded Up Robust Features** which introduced a new algorithm called SURF. As name suggests, it is a speeded-up version of SIFT.
In SIFT, Lowe approximated Laplacian of Gaussian with Difference of Gaussian for finding scale-space. SURF goes a little further and approximates LoG with Box Filter. Below image shows a demonstration of such an approximation. One big advantage of this approximation is that, convolution with box filter can be easily calculated with the help of integral images. And it can be done in parallel for different scales. Also the SURF rely on determinant of Hessian matrix for both scale and location.
![surf_boxfilter](https://docs.opencv.org/3.0-beta/_images/surf_boxfilter.jpg)

For orientation assignment, SURF uses wavelet responses in horizontal and vertical direction for a neighbourhood of size 6s. Adequate guassian weights are also applied to it. Then they are plotted in a space as given in below image. The dominant orientation is estimated by calculating the sum of all responses within a sliding orientation window of angle 60 degrees. Interesting thing is that, wavelet response can be found out using integral images very easily at any scale. For many applications, rotation invariance is not required, so no need of finding this orientation, which speeds up the process. SURF provides such a functionality called Upright-SURF or U-SURF. It improves speed and is robust upto  $\pm 15^{\circ}$. OpenCV supports both, depending upon the flag,  **upright**. If it is 0, orientation is calculated. If it is 1, orientation is not calculated and it is more faster.
			![](https://docs.opencv.org/3.0-beta/_images/surf_orientation.jpg)
For feature description, SURF uses Wavelet responses in horizontal and vertical direction (again, use of integral images makes things easier). A neighbourhood of size 20sX20s is taken around the keypoint where s is the size. It is divided into 4x4 subregions. For each subregion, horizontal and vertical wavelet responses are taken and a vector is formed like this, $v = (\sum d_x , \sum d_y, \sum |d_x|),\sum|d_y|)$. This when represented as a vector gives SURF feature descriptor with total 64 dimensions. Lower the dimension, higher the speed of computation and matching, but provide better distinctiveness of features.

For more distinctiveness, SURF feature descriptor has an extended 128 dimension version. The sums of$d_x$ and $|d_x|$ are computed separately for $d_y < 0$ and $d_y \geq 0$. Similarly, the sums of $d_y$ and $|d_y|$ are split up according to the sign of $d_x$ , thereby doubling the number of features. It doesn’t add much computation complexity. OpenCV supports both by setting the value of flag extended with 0 and 1 for 64-dim and 128-dim respectively (default is 128-dim)

Another important improvement is the use of sign of Laplacian (trace of Hessian Matrix) for underlying interest point. It adds no computation cost since it is already computed during detection. The sign of the Laplacian distinguishes bright blobs on dark backgrounds from the reverse situation. In the matching stage, we only compare features if they have the same type of contrast (as shown in image below). This minimal information allows for faster matching, without reducing the descriptor’s performance.
![](https://docs.opencv.org/3.0-beta/_images/surf_matching.jpg)
In short, SURF adds a lot of features to improve the speed in every step. Analysis shows it is 3 times faster than SIFT while performance is comparable to SIFT. SURF is good at handling images with blurring and rotation, but not good at handling viewpoint change and illumination change.
## FAST Algorithm for Corner Detection
**Goal**
In this paper,
-   We will understand the basics of FAST algorithm
-   We will find corners using OpenCV functionalities for FAST algorithm.

**Theory**
We saw several feature detectors and many of them are really good. But when looking from a real-time application point of view, they are not fast enough. One best example would be SLAM (Simultaneous Localization and Mapping) mobile robot which have limited computational resources.

As a solution to this, FAST (Features from Accelerated Segment Test) algorithm was proposed by Edward Rosten and Tom Drummond in their paper **Machine learning for high-speed corner detection** in 2006 (Later revised it in 2010). A basic summary of the algorithm is presented below. Refer original paper for more details (All the images are taken from original paper).

### Feature Detection using FAST
1.  Select a pixel $p$ in the image which is to be identified as an interest point or not. Let its intensity be  $I_p$. 
2.  Select appropriate threshold value  $t$
3.  Consider a circle of 16 pixels around the pixel under test. (See the image below)
![fast_speedtest](https://docs.opencv.org/3.0-beta/_images/fast_speedtest.jpg)
4. Now the pixel $p$ is a corner if there exists a set of n contiguous pixels in the circle (of 16 pixels) which are all brighter than $I_p + t$, or all darker than $I_p − t$. (Shown as white dash lines in the above image). n was chosen to be 12.
5. A **high-speed test** was proposed to exclude a large number of non-corners. This test examines only the four pixels at 1, 9, 5 and 13 (First 1 and 9 are tested if they are too brighter or darker. If so, then checks 5 and 13). If $p$ is a corner, then at least three of these must all be brighter than $I_p + t$ or darker than $I_p − t$. If neither of these is the case, then p cannot be a corner. The full segment test criterion can then be applied to the passed candidates by examining all pixels in the circle. This detector in itself exhibits high performance, but there are several weaknesses:
> - It does not reject as many candidates for n < 12.
> - The choice of pixels is not optimal because its efficiency depends on ordering of the questions and distribution of corner appearances.
> - Results of high-speed tests are thrown away.
> - Multiple features are detected adjacent to one another.

First 3 points are addressed with a machine learning approach. Last one is addressed using non-maximal suppression.

### Machine Learning a Corner Detector

1.  Select a set of images for training (preferably from the target application domain)
    
2.  Run FAST algorithm in every images to find feature points.
    
3.  For every feature point, store the 16 pixels around it as a vector. Do it for all the images to get feature vector $p.$
4.  Each pixel (say  $x$ in these 16 pixels can have one of the following three states:
$$
S_{p->x} = \left\{
\begin{aligned}
d, \  \  \ \  \  \ & I_{p->x} \leq I_p -t & (darker) \\
s,\  \  \ \  \  \  & I_p-t < I_{p->x} < I_p+t & (similar)\\ 
b,\  \  \ \  \  \  & I_p+t \geq I_{P->x}&(brighter)
\end{aligned}
\right.
$$
5. Depending on these states, the feature vector $p$ is subdivided into 3 subsets, $p_d, p_s, p_b$.
6. Define a new boolean variable, $K_p$, which is true if $p$ is a corner and false otherwise.
7. Use the ID3 algorithm (decision tree classifier) to query each subset using the variable $K_p$ for the knowledge about the true class. It selects the $x$ which yields the most information about whether the candidate pixel is a corner, measured by the entropy of $K_p$.
8. This is recursively applied to all the subsets until its entropy is zero.
9. The decision tree so created is used for fast detection in other images.
 
 ### Non-maximal Suppression
Detecting multiple interest points in adjacent locations is another problem. It is solved by using Non-maximum Suppression.
> - Compute a score function, $V$ for all the detected feature points. $V$ is the sum of absolute difference between $p$ and 16 surrounding pixels values.
> - Consider two adjacent keypoints and compute their $V$ values.
> - Discard the one with lower $V$ value.

### Summary
- It is several times faster than other existing corner detectors.
- But it is not robust to high levels of noise. It is dependant on a threshold.


## BRIEF (Binary Robust Independent Elementary Features)
**Goal**
In this chapter
- We will see the basics of BRIEF algorithm.

**Theory**
We know SIFT uses 128-dim vector for descriptors. Since it is using floating point numbers, it takes basically 512 bytes. Similarly SURF also takes minimum of 256 bytes (for 64-dim). Creating such a vector for thousands of features takes a lot of memory which are not feasible for resouce-constraint applications especially for embedded systems. Larger the memory, longer the time it takes for matching.

But all these dimensions may not be needed for actual matching. We can compress it using several methods like PCA, LDA etc. Even other methods like hashing using LSH (Locality Sensitive Hashing) is used to convert these SIFT descriptors in floating point numbers to binary strings. These binary strings are used to match features using Hamming distance. This provides better speed-up because finding hamming distance is just applying XOR and bit count, which are very fast in modern CPUs with SSE instructions. But here, we need to find the descriptors first, then only we can apply hashing, which doesn’t solve our initial problem on memory.

BRIEF comes into picture at this moment. It provides a shortcut to find the binary strings directly without finding descriptors. It takes smoothened image patch and selects a set of $n_d$(x,y) location pairs in an unique way (explained in paper). Then some pixel intensity comparisons are done on these location pairs. For eg, let first location pairs be $p$ and $q$. If $I(p) < I(q)$ , then its result is 1, else it is 0. This is applied for all the $n_d$ location pairs to get a $n_d$-dimensional bitstring.

This $n_d$ can be 128, 256 or 512. OpenCV supports all of these, but by default, it would be 256 (OpenCV represents it in bytes. So the values will be 16, 32 and 64). So once you get this, you can use Hamming Distance to match these descriptors.

One important point is that BRIEF is a feature descriptor, it doesn’t provide any method to find the features. So you will have to use any other feature detectors like SIFT, SURF etc. The paper recommends to use CenSurE which is a fast detector and BRIEF works even slightly better for CenSurE points than for SURF points.

In short, BRIEF is a faster method feature descriptor calculation and matching. It also provides high recognition rate unless there is large in-plane rotation.

## ORB (Oriented FAST and Rotated BRIEF)
**Goal**
In this chapter,
-   We will see the basics of ORB

**Theory**
As an OpenCV enthusiast, the most important thing about the ORB is that it came from “OpenCV Labs”. This algorithm was brought up by Ethan Rublee, Vincent Rabaud, Kurt Konolige and Gary R. Bradski in their paper  **ORB: An efficient alternative to SIFT or SURF**  in 2011. As the title says, it is a good alternative to SIFT and SURF in computation cost, matching performance and mainly the patents. Yes, SIFT and SURF are patented and you are supposed to pay them for its use. But ORB is not !!!

ORB is basically a fusion of FAST keypoint detector and BRIEF descriptor with many modifications to enhance the performance. First it use FAST to find keypoints, then apply Harris corner measure to find top N points among them. It also use pyramid to produce multiscale-features. But one problem is that, FAST doesn’t compute the orientation. So what about rotation invariance? Authors came up with following modification.

It computes the intensity weighted centroid of the patch with located corner at center. The direction of the vector from this corner point to centroid gives the orientation. To improve the rotation invariance, moments are computed with x and y which should be in a circular region of radius  $r$, where   $r$  is the size of the patch.

Now for descriptors, ORB use BRIEF descriptors. But we have already seen that BRIEF performs poorly with rotation. So what ORB does is to “steer” BRIEF according to the orientation of keypoints. For any feature set of $n$ binary tests at location $(x_i,y_i)$, define a $2 \times n$ matrix, $S$ which contains the coordinates of these pixels. Then using the orientation of patch, $\theta$, its rotation matrix is found and rotates the $S$ to get steered(rotated) version $S_{\theta}$.

ORB discretize the angle to increments of $2 \pi / 30$ (12 degrees), and construct a lookup table of precomputed BRIEF patterns. As long as the keypoint orientation $\theta$ is consistent across views, the correct set of points $S_{\theta}$ will be used to compute its descriptor.

BRIEF has an important property that each bit feature has a large variance and a mean near 0.5. But once it is oriented along keypoint direction, it loses this property and become more distributed. High variance makes a feature more discriminative, since it responds differentially to inputs. Another desirable property is to have the tests uncorrelated, since then each test will contribute to the result. To resolve all these, ORB runs a greedy search among all possible binary tests to find the ones that have both high variance and means close to 0.5, as well as being uncorrelated. The result is called **rBRIEF**.

For descriptor matching, multi-probe LSH which improves on the traditional LSH, is used. The paper says ORB is much faster than SURF and SIFT and ORB descriptor works better than SURF. ORB is a good choice in low-power devices for panorama stitching etc.

# Machine Learning
## K-Nearest Neighbour
Learn to use kNN for classification Plus learn about handwritten digit recognition using kNN
### Understanding k-Nearest Neighbour
**Goal** 
In this chapter, we will understand the concepts of k-Nearest Neighbour (kNN) algorithm.
**Theory**
kNN is one of the simplest of classification algorithms available for supervised learning. The idea is to search for closest match of the test data in feature space. We will look into it with below image.
![](https://docs.opencv.org/3.0-beta/_images/knn_theory.png)

In the image, there are two families,  Blue Squares and Red Triangles. We call each family as  **Class**. Their houses are shown in their town map which we call  feature space.  _(You can consider a feature space as a space where all datas are projected. For example, consider a 2D coordinate space. Each data has two features, x and y coordinates. You can represent this data in your 2D coordinate space, right? Now imagine if there are three features, you need 3D space. Now consider N features, where you need N-dimensional space, right? This N-dimensional space is its feature space. In our image, you can consider it as a 2D case with two features)_.

Now a new member comes into the town and creates a new home, which is shown as green circle. He should be added to one of these Blue/Red families. We call that process,  **Classification**. What we do? Since we are dealing with kNN, let us apply this algorithm.

One method is to check who is his nearest neighbour. From the image, it is clear it is the Red Triangle family. So he is also added into Red Triangle. This method is called simply  **Nearest Neighbour**, because classification depends only on the nearest neighbour.

But there is a problem with that. Red Triangle may be the nearest. But what if there are lot of Blue Squares near to him? Then Blue Squares have more strength in that locality than Red Triangle. So just checking nearest one is not sufficient. Instead we check some k nearest families. Then whoever is majority in them, the new guy belongs to that family. In our image, let’s take k=3, ie 3 nearest families. He has two Red and one Blue (there are two Blues equidistant, but since k=3, we take only one of them), so again he should be added to Red family. But what if we take k=7? Then he has 5 Blue families and 2 Red families. Great!! Now he should be added to Blue family. So it all changes with value of k. More funny thing is, what if k = 4? He has 2 Red and 2 Blue neighbours. It is a tie !!! So better take k as an odd number. So this method is called **k-Nearest Neighbour** since classification depends on k nearest neighbours.

Again, in kNN, it is true we are considering k neighbours, but we are giving equal importance to all, right? Is it justice? For example, take the case of k=4. We told it is a tie. But see, the 2 Red families are more closer to him than the other 2 Blue families. So he is more eligible to be added to Red. So how do we mathematically explain that? We give some weights to each family depending on their distance to the new-comer. For those who are near to him get higher weights while those are far away get lower weights. Then we add total weights of each family separately. Whoever gets highest total weights, new-comer goes to that family. This is called **modified kNN**.

So what are some important things you see here?
> -   You need to have information about all the houses in town, right? Because, we have to check the distance from new-comer to all the existing houses to find the nearest neighbour. If there are plenty of houses and families, it takes lots of memory, and more time for calculation also.
> -   There is almost zero time for any kind of training or preparation.

## Support Vector Machines (SVM)
## K-Means Clustering
