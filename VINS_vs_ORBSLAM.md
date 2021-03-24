# *Compare VINS and ORB_SLAM*

## *VINS*



## *ORB_SLAM*

### ORB_SLAM 1~3


### ORB_SLAM VI
#### ORI Visual Part and Imu part

$$\pi(X_c=\left[\begin{matrix}
    f_u\frac{X_c}{Z_c}+c_u \\ 
    f_v\frac{Y_c}{Z_c}+c_v
\end{matrix} \right]), X_c=[X_c, Y_c, Z_c]^T$$

$$R^{k+1}_{wb}=R^k_{wb}Exp((w^k_b-b^k_g)\Delta t)$$

$${}_wv^{k+1}={}_wv^{k}_b+g_w\Delta t+R^k_{wb}(a^k_b-b^k_a)\Delta a$$

$${}_wP^{k+1}_b={}_wp^k_b+{}_wv^k_b\Delta t+\frac{1}{2} g_w \Delta t^2+\frac{1}{2}R^k_{wb}(a^k_b-b^k_a)\Delta t^2$$

*use the recent IMU preintegration described like this:*

$$R^{i+1}_wb=R^i_{wb}\Delta R_{i,i+1}Exp((J^g_{\Delta R}b^i_a))$$

$${}_wv^{i+1}_b={}_wv^i_b+g_w\Delta t_{i,i+1}+R^i_{wb}(\Delta v_{i,i+1}+J^g_{\Delta v}b^i_g+J^a_{\Delta v}b^i_a)$$

$${}_wP^{k+1}_b={}_wp^k_b+{}_wv^k_b\Delta t_{i,i+1}+\frac{1}{2} g_w \Delta t^2_{i,i+1}+R^k_{wb}(\Delta p_{i,i+1}+J^g_\Delta p b^i_g+J^a_{\Delta p})$$


*the cost function should be min*

*element is $\theta$*
$$\theta = \left\{R^j_{wb}, {}_wp^j_b, {}_wv^j_b,b^j_g, b^j_a\right\}$$

$$\theta^*=argmin\theta \left(\sum_kE_{proj}(k,j)+E_{IMU}(i,j) \right)$$

*视觉产生的误差如下：*
$$E_{proj}(k,j)=\rho((x^k-\pi(X^k_c))^T\Sigma_k(x^k-\pi(X^k_c)))$$

其中，$X^k_c$ is like this:

$$X^k_c=R_{cb}R^j_{bw}(X^k_w-{}_wp^j_b)+{}_cp_b$$

*IMU产生的误差如下：*
$$E_{IMU}(i,j)=\rho ([e^T_Re^T_ve^T_p])\Sigma_I[e^T_Re^T_ve^T_p]^T+\rho (e^T_b\Sigma_Re_b)$$

$$e_R = Log((\Delta R_{ij}Exp(J^g_{\Delta R}b^j_g))^TR^i_{bw}R^j_{wb})$$

$$e_v = R^i_{bw}({}_wv^j_b-{}_wv^i_b-g_w\Delta t_{ij})-(\Delta v_{ij}+J^g_\Delta v b^i_j + J^a_\Delta v b^j_a)$$

$$e_p = R^i_{bw}({}_wp^j_b-{}_wp^i_b-{}_wv^i_b\Delta t_{ij}-\frac{1}{2}g_w\Delta t^2_{ij})-(\Delta p_{ij}+J^g_{\Delta p}b^j_g+J^a_{\Delta p}b^j_a)$$

$$e_b = b^j-b^i$$


if(Assuming no map update, the next frame $j+1$ wil be optimized with a link to frame J and using the prior computed at the end of the previous optimization.)

$$\theta = \{R^j_{wb}, p^j_w, v^j_w, b^j_g, b^j_a, R^j+1_{wb}, p^j+1_w, v^j+1_w, b^j+1_g, b^j+1_a \}$$

$$\theta^*=argmin\theta \left(\sum_kE_{proj}(k,j)+E_{IMU}(i,j) + E_{prior}(j)\right)$$

$$E_{prior}(j) = \rho [e^T_Re^T_ve^T_pe^T_b]\Sigma_p[e^T_Re^T_ve^T_pe^T_b]^T$$
$$e_R=Log(\bar{R}^j_{bw}R^j_{wb}), e_v={}_w\bar{v}^j_b-{}_wv^j_b$$
$$e_p={}_w\bar{p}^j_b-{}_wp^j_b,e_b=\bar{b}^j-b^j$$

$(\bar{*})$ is estimate values



### *IMU initialzation*
$$argmin_{b_g}\sum^{N-1}_{i=1}||log((\Delta R_{i,i+1})Exp(J^g_\Delta Rb_g))^TR^{i+1}_{bw}R^i_{wb}||$$


#### *scale and Gravity Approximation*

***here is the most important point in the VIO-ORB_SLAM***

$s$ is a scale factor when transforming between camera C and IMU B coordiante system:

${}_wp_b$ is a vector in the w coordinate

$${}_wp_b=s{}_wp_c+R_{wc}{}_cp_b$$

$$s{}_wp^{i+1}_c=s{}_wp^i_{c}+{}_wv^i_b\Delta t_{i,i+1}+\frac{1}{2}g_w\Delta t^w_{i,i+1}+R^i_{wb}\Delta p_{i,i+1}+(R^i_{wc}-R^{i+1}_{wc}){}_cp_b$$

$$s{}_wp_c^{i+1}-\frac{1}{2}g_w\Delta t^2_{i,i+1}=s{}_wp^i_{c}+{}_wv^i_b\Delta t_{i,i+1}+R^i_{wb}\Delta p_{i,i+1}+(R^i_{wc}-R^{i+1}_{wc}){}_cp_b$$
<!-- 当存在多个关键帧存在时，可以联立求解 -->

how to estimate the $s$ and $g_w$ by solving a linear system of equations on those variables. 
 
$$[\lambda](i), \beta(i)]\left[\begin{matrix}
    s \\ g_w
\end{matrix}\right]$$

keyframe $i$,$i+1$,$i+2$ as 1,2,3 for clarity of notation, we have.

$$\lambda(i)=({}_wp^2_c-{}_wp^1_c)\Delta t_{23} - ({}_wp^3_c - {}_wp^2_c)\Delta t_{12}$$

$$\beta(i) = \frac{1}{2}I_{3\times 3}(\Delta t^2_{12}\Delta t_{23}+\Delta t^2_{23}\Delta t_{12})$$

$$\gamma(i)=(R^2_wc - R^1_wc){}_cp_b\Delta t_{23} - (R^2_{wc}\\
            -R^2_{wc}{}_cp_b\Delta t_{12}+R^2_{wb}\Delta p_{23}\Delta t_{12} \\+R^1_{wb}\Delta v_{12}\Delta t_{12}\Delta t_{23}-R^1_{wb}\Delta p_{12}\Delta t_{23}$$

#### Acc bias estimation and Scale and gravity direction refinement

gravity magnitude $G$, 

$$R_{wi} = Exp(\hat{v}\theta), \hat{v}=\frac{\hat{g_i}\times \hat{g_w}}{||\hat{g_i}\times \hat{g_w}||}, \theta = atan2(\hat{g_i}\times \hat{g_w}, \hat{g_i},\hat{g_w})$$

express now the gravity vector as:
$$g_w = R_{wi}\hat{g}_iG$$