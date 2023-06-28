# SLAMPortfolio
This is the Implementation of SLAM using Kitti Data with C++

![bandicam 2023-06-25 10-38-09-472](https://github.com/WD4715/SLAMPortfolio/assets/117700793/4afaea4f-84f0-4262-a649-811bf9b1a92c)


## 1. FrontEnd Process

### 1. Keypoint Detector

I used ***Keypoint Detector(cv::GFTTDetector)***. This functionn is the ***Lucas-kanade algorithm.***
Lucas-Kanade Algorithm is a method of describing pixels' movement between images.  
Now considera 2D pixel. Its coordinates at time t are x, y. Due to the movement of the camera, its image coordinates will change. ***We want to estimate the position of this pixel at other times***.

- Assumption

$$
\mathcal{I(x+dx, y+dy, t+dt)}= \mathcal{I(x, y, t)}
$$

This means that if dx, dy, dt is too small, so the value doesn't change.  
We can solve the above equation using ***Taylor Expension.***

$$
I(x+dx, y+dy, t+dt) \approx I(x, y, t) + {\partial{I} \over \partial{x}}dx + {\partial{I} \over \partial{y}}dy + {\partial{I} \over \partial{t}}dt
$$

Because we assume that the brightness does not chage, so we have:

$$
{\partial{I}\over\partial{x}}dx + {\partial{I} \over \partial{y}}dy + {\partial{I} \over \partial{t}}dt = 0 
$$

this is the same with :

$$
{\partial{I} \over \partial{x}}{dx \over dt} + {\partial{I} \over \partial{y}}{dy \over dt}  =- {\partial{I} \over \partial{t}}
$$

Let's look at the above equation:

${dx \over dt}$ is the speed of the pixel on the x-axis. We can denote this ***u***  
${\partial{I} \over \partial{x}}$ is the gradient of the image in x direction at this point. We can denote this ***I<sub>x</sub>.***

So the above equation will be:

$$
[I_x, I_y] 
\begin{pmatrix} 
   u  \\
   v  \\
\end{pmatrix} =
-I_t
$$

What we want is to calculate the motion u, v of the pixel, but this formula is a linear equation with two variables, and we cannot find ***u, v*** by a single pixel. Therefore, additional constraints are needed to calculate u, v. ***In LK optical flow, we also assume the pixels in a certain window have the same motion.***  

$$
\begin{pmatrix} 
   u  \\
   v  \\
\end{pmatrix}^{*}=
-(A^{T}|A)^{-1}A^{T}b
$$

we know the notations:

$$
A=
\begin{pmatrix} 
   [I_x, I_y]_1  \\
   ...  \\
   [I_x, I_y]_k \\
\end{pmatrix}
$$

$$
b=
\begin{pmatrix} 
   I_{t1}  \\
   ...  \\
   I_{tk} \\
\end{pmatrix}
$$

### 2. Triangularation

Then the next step is to get **the Spatial Position of the FEATURE POINT** using the camera pose.

The first thing to get the spatial position of feature point is **to estimate the depths of the map points**.

--> How to? **"Triangulation"**
Triangulation refers to observing **the same landmark point at different locations** and **determining the distance of the landmark point** from the observed locations.

Let's define x<sub>1</sub> and x<sub>2</sub> be the normalized coordinates of two feature points


$$
s_{2}x_{2}=s_{1}Rx_{1}+t
$$

s<sub>1</sub> and s<sub>2</sub> denote the depth of two feature points. And R is Rotation matrix and t is transition vector.
Let's multiply [x<sub>2</sub>]<sub>x</sub> at the left side. 

Then We can get the following equations:

$$
s_2 [x_2]_{x} x_2 = s_1 [x_2]_x Rx_1 + [x_2]_x t
$$

Look at the above equation. Then we know that the left side of equation will be zero because of the outer product with x<sub>2</sub> itself will be zero.
Then above equation will be below :


$$
0 = s_1 [x_2]_x Rx_1 + [x_2]_x t
$$


- The right side of can be regarded as an equation of s<sub>1</sub>, and s<sub>1</sub> can be obtained directly from it.
And **Meaning of calculating of the depth of keypoint is the same as the Spatial position of the feature point.**

## 2. BackEnd Process

### 1. Kalman Filter
The visual odometry(Front-End) has a short memory, but we hope that the system can maintain ***the entire motion trajectory in an optimal state for a long time.***  
Therefore, in the backend optization, we usually consider the problem ofr state estimation for longer time and not only use the past information to update our current state ***but also use future information to update ourselves.***
we have the ***poses from x_<sub>0</sub>, ..., x<sub>N</sub>*** and ***observation y<sub>1</sub>, ..., y<sub>M</sub>.***  

$$
x_{k}=f(x_{k-1}, u_k) + w_k, z_{k, j}=h(y_j, x_k) + v_{k, j}
$$

We know that every measurement is affected by noise, so the poses ***x*** and landark ***y*** here are regarded as ***random variables***.  
Therefore, the question becomes: when I have some motion data ***u*** and observation data ***z***, how to determine the state ***x*** and larndmarks ***y***'s distributuon?  

***We hope to use the data from 0 to k to estimate the current state distribution:***  


$$
P(x_k|x_0, u_{0:k}, z_{1:k}) \propto P(z_k|x_k)P(x_k|x_0, u_{0:k}, z_{1:k-1})
$$

$$
P(x_k|x_0, u_{0:k}, z_{1:k-1})=\int{P(x_k|x_{k-1}, x_0, u_{0:k}, z_{1:k-1})P(x_{k-1}|x_0, u_{1:k}, z_{1:k-1})}dx_{k-1}
$$

Because of ***Markov Property***, We can modify this equation like below :

$$
P(x_k|x_{k-1}, x_0, u_{0:k}, z_{1:k-1})=P(x_k|x_{k-1}, u_{k}) 
$$

$$
P(x_{k-1}|x_0, u_{1:k}, z_{1:k-1})}=P(x_{k-1}|x_0, u_{1:k-1}, z_{1:k-1})}
$$

In other words, we only need to maintain the current state estimation and update it increm
