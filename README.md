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
I(x+dx, y+dy, t+dt) \approx I(x, y, t) + {\partial{I} \ over \parital{x}}dx + {\partial{I} \ over \parital{y}}dy + {\partial{I} \ over \parital{t}}dt
$$

Because we assume that the brightness does not chage, so we have:

$$
{\partial{I} \over \partial{x}}dx + {\partial{I} \over \partial{y}}dy + {\partial{I} \over \partial{t}}dt =0 
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
,b=
\begin{pmatrix} 
   I_{t1}  \\
   ...  \\
   I_{tk} \\
\end{pmatrix}
$$

## 2. BackEnd Process
