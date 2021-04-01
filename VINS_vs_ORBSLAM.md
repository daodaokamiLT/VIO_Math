# *Compare VINS and ORB_SLAM*



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




## *VINS*
$$P^{w}_{b_{k+1}}=P^w_{b_k}+v^w_{b_k}\Delta t_k + \int\int_{t\in [t_k, t_{k+1}]}(R^w_t(\hat{a}_t-b_{a_t}-n_a)-g^w)dt^2$$

$$v^w_{b_{k+1}}=v^w_{b_k}+\int_{t\in [t_k, t_{k+1}]}(R^w_t(\hat{a}_t-b_{a_t}-n_a)-g^w)dt$$

$$q^w_{b_{k+1}}=q^w_{b_k}\bigotimes \int_{t_k, t_{k+1}}\frac{1}{2}\omega(\hat(w)_t - b_{w_t}-n_w)q^{b_k}_tdt$$

where 

$$\omega = \left[\begin{matrix}
    -w_{\times} & w \\ 
    -w^T & 0=
\end{matrix} \right], w_{times}=\left[\begin{matrix}
    0 & -w_z & w_y \\
    w_z & 0 & -w_x \\
    -w_y & w_x & 0
\end{matrix}\right]$$

at the begin $\alpha^{b_l}_{b_k}$,$\beta^{b_k}_{b_k}$,$\gamma^{b_k}_{b_k}$ is 0, 0, Identity quaternion. these values is propagated step by step as follow.

$$\hat{\alpha}^{b_k}_{i+1}=\hat{\alpha}^{b_k}_i+\hat{\beta}^{b_k}_i\delta t+\frac{1}{2}R(\hat{\gamma}^{b_k}_i)(\hat{a}_i-b_{a_i})\delta t^2$$

$$\hat{\beta^{b_k}_{i+1}}=\hat{\beta}^{b_k}_i+R(\hat{\gamma}^{b_k}_i)(\hat{a}_i-b_{ai}\delta t)$$

$$\hat{\gamma}^{b_k}_{i+1} = \hat{\gamma}^{b_k}_i\bigotimes\left[\begin{matrix}
    1 \\ \frac{1}{2}(\hat{w}_i - b_{wi}\delta t)
\end{matrix} \right]$$

$$\gamma^{b_k}_t\approx\hat{\gamma}^{b_k}_t\bigotimes\left[\begin{matrix}
    1 \\ \frac{1}{2}\delta \theta^{b_t}_t
\end{matrix}\right]$$
the $\delta \theta^{b_k}_t$ is three-dimensinal small perturbation.


the error terms is 
$$\left[\begin{matrix}
    \delta \dot{\alpha}^{b_k}_t \\ 
    \delta \dot{\beta}^{b_k}_t \\ 
    \delta \dot{\theta}^{b_k}_t \\ 
    \delta \dot{b}_t{a_t} \\ 
    \delta \dot{b}_{w_t} \\ 
\end{matrix} \right] = \left[\begin{matrix}
    0 &I &0 &0 &0 \\ 
    0 &0 &-R^{b_k}_tvec(\hat{a}_t - b_{a_t}) &-R^{b_k}_t &0 \\
    0 &0 &-vec(\hat{w}_t-b_{w_t}) &0 &-I\\
    0 &0 &0 &0 &0 \\
    0 &0 &0 &0 &0
\end{matrix} \right] \left[\begin{matrix}
    \delta {\alpha}^{b_k}_t \\ 
    \delta {\beta}^{b_k}_t \\ 
    \delta {\theta}^{b_k}_t \\ 
    \delta {b}_t{a_t} \\ 
    \delta {b}_{w_t} \\ 
\end{matrix} \right] +\left[\begin{matrix}
    0 &0 &0 &0 \\
    -R^{b_t}_t &0 &0 &0 \\
    0 &-I &0 &0 \\
    0 &0 &-I &0 \\
    0 &0 &0 &0 
\end{matrix} \right]\left[\begin{matrix}
    n_a \\ n_w \\ n_{b_a}\\ n_{b_w}
\end{matrix} \right]$$



### VINS init is the most important partion







linear acceleration $\hat{a}$ and angular velocity $\hat{w}$ as follows:
$$R^{b_k}_wp^w_{b_{k+1}}=R^{b_k}_w(p^w_{b_k}+v^w_{b_k}\Delta t -\frac{1}{2}g^2\Delta t^2_k)+\alpha^{b_k}_{b_{k+1}}$$



$$R^{b_k}_wv^w_{b_{k+1}}=R^{b_{k+1}}_w(v^w_{b_k}-g^w_\Delta t_k)+\beta^{b_k}_{b_{k+1}}$$

$$q^{b_k}_w\bigotimes q^w_{b_{k+1}}=\gamma^{b_k}_{b_{k+1}}$$

where:
$$\alpha^{b_k}_{b_{k+1}}=\int\int_{t\in[t_k, t_{k+1}]}R^{b_k}_t(\hat{a}_t-b_{a_t}-n_a)dt^2$$

$$\beta^{b_k}_{b_{k+1}}=\int_{t\in[t_k, t_{k+1}]}R^{b_k}_t(\hat{a}_t-b_{a_t}-n_a)dt$$

$$\gamma^{b_k}_{b_{k+1}}=\int_{t\in[t_k, t_{k+1}]}\frac{1}{2}\omega(\hat{w}_t-b_{a_t}-n_w)dt$$

### how to estimate the errors

## Estimator Initialzation
### sliding window vision-only sfm



### how the preintergate work
#### basic quaternion
$$exp(\phi^\times)=I+\frac{sin(||\phi||)}{||\phi||}\phi^\times+\frac{1-cos(||\phi||)}{||\phi||}(\phi^\times)^2$$

当$\phi$是小量时，有一阶近似：
$$exp(\phi^\times)\approx I+\phi^\times$$

对数映射，将$SO(3)$中的元素映射到$so(3)$上：
$$log(R)=\frac{\psi（R-R^T）}{2sin(\psi)}$$

where 
$$\psi=cos^{-1}(\frac{tr(R)-1}{2})$$

有$vec(\log(R))=\psi a$,其中$\psi$为旋转角，$a$为旋转轴的单位矢量，有$a=vec(\left[\frac{R-R^T}{2sin(\psi)}\right])$

$$R(t+\Delta t)=R(t)Exp((\tilde{w}(t)-b^g(t)-\eta^{gd}(t))\Delta t)$$

$$v_(t+\Delta t)=v(t)+a^w(t))\Delta t\\
=v(t)+R(t)(\tilde{f}(t)-b_a(t)-\eta_{ad}(t))\Delta t + g\Delta t$$

$$p(t+\Delta t) = p(t)+v(t)\Delta t+\frac{1}{2}a^w(t)\Delta t^2 \\ = p(t) + v(t) \Delta t+ \frac{1}{2}\left[R(t)(\tilde{f}(t)-b_a^t-\eta_{ad}(t))+g \right]\Delta t^2 \\
= p(t)+v(t)\Delta t + \frac{1}{2}\left[R(t)(\tilde{f}(t)-b_a(t)-\eta_{ad}(t)) \Delta t^2\right]$$


噪声项是$\eta_{gd}$,$\eta_{ad}$他们与连续噪声项目，$\eta_g$和$\eta_a$是不同的，存在关系：
$$Cov(\eta_{gd}(t)) = \frac{1}{\Delta t}Cov(\eta_g(t))$$
$$Cov(\eta_{ad}(t)) = \frac{1}{\Delta t}Cov(\eta_a(t))$$


用 $i$到$j-1$时间段内的所有imu数据进行预积分，当$i$时刻状态完成初始化后，直接更新得到$k=j$时刻的状态。

$$R_j=R_i\prod^{j-1}_{k=i} Exp((\tilde{w}_k-b_k^g-\eta^{gd}_k)\Delta t)$$

$$v_j=v_i+g\Delta t_{ij}+\sum^{j-1}_{k=i}R_k(\tilde{f}_k-b^a_k-\eta^{ad}_k)\Delta t$$

$$p_j=p_i+\sum^{j-1}_{k=i}\left[v_k\Delta t+\frac{1}{2}g\Delta t^2+\frac{1}{2}R_k(\tilde{f}_k-b_k^a-\eta_k^{ad})\Delta t^2 \right]$$

修改如上公式如下，转换成预积分模式：
$$\Delta R_{ij} = R^T_iR_j \\
                = \prod^{j-1}_{k=i}Exp((\tilde{w}_k-b^g_k-\eta^{gd}_k)\Delta t)$$

$$\Delta v_{ij}=R^T_i(v_j-v_i-g\Delta t_{ij}) how-to-get-this-result$$
prove:
$$
    \Delta v_{ij} = (v_j - v_i) = g\Delta t_{ij} + \sum^{j-i}_{k=i}R_k(\tilde{f_k}-b^a_k-\eta^{ad}_k)\Delta t\\ 
    (split) = g(\Delta t_{i,i+1}+\Delta t_{i+1,i+2}+\cdots)\\+R_i(\tilde{f_i}-b^a_i-\eta^{ad}_i)\Delta t_{i,i+1}\\+R_{i+1}(\tilde{f_{i+1}}-b^a_{i+1}-\eta^{ad}_{i+1})\Delta t_{i+1,i+2}+\cdots \\
    = g(\Delta t_{i,i+1}+\Delta t_{i+1,i+2}+\cdots) \\
    + R_i(\tilde{f_i}-b^a_i-\eta^{ad}_i)\Delta t_{i,i+1})\\
    + R_i*\Delta R_{i,i+1}(\tilde{f_{i+1}}-b^a_{i+1}-\eta^{ad}_{i+1})\Delta t_{i+1,i+2})+\cdots$$

not proved!!!

$$\Delta v_{ij}= R^T_i(v_j-v_i-g\Delta t_{ij}) how-to-get-this-result\\
                = R^T_i(v_j-v_i-g\Delta t_{ij}) \\
                = R^T_i \sum^{j-1}_{k=i}R_k(\tilde{f}_k-b_k^a-\eta^{ad}_k)\Delta t \\
                = \sum^{j-1}_{k=i}\Delta R_{ik}(\tilde{f_k}-b^a_k-\eta^{ad}_k)\Delta t$$

<!-- the speed can be calculated by the split the g and the init i status-->

so $p$ can be cal like these:

$$\Delta p_{ij} = R^T_i(p_j - p_i-v_i\Delta t_{ij} - \frac{1}{2}g\Delta t^2)\\
= R^T_i(\sum^{j-1}_{k=i}\left[v_k\Delta t + \frac{1}{2}g\Delta t^2 +\frac{1}{2}R_k(\tilde{f}_k-b^a_k-\eta^{ad}_k)\Delta t^2 \right]-v_i\Delta t_{ij}-\frac{1}{2}g\Delta t_{ij}^2) \\ 
= use-the-function-\Delta v_{ij}\\
=R^T_i(\sum^{j-1}_{k=i}\left[v_{ik}\Delta t +\frac{1}{2}R_k(\tilde{f}_k-b^a_k-\eta^{ad}_k)\Delta t^2  \right])$$

<!-- 其中最重要的是在融合进v & g的过程中的变化，用到了 -->

通过简单的变形，将$\Delta v_{ij}$和$\Delta p_{ij}$转换成无$v_i$无关的样式。

#### 预积分测量值和测量噪声
将噪声项$\eta^{gd}_k$，$\eta^{ad}_k$从预计分中分离，是的预积分测量值，由IMU测量数据计算得到。认为预积分计算区间内的bias相等，即$b^g_i=b^g_{i+1}=\cdots=b^g_j$

$$\Delta R_{ij} = \prod^{j-1}_{k=i}Exp((\tilde{w}_k-b^g_i)\Delta t - \eta^{gd}_k\Delta t) \\ 
\approx \prod^{j-1}_{k=i} \left\{Exp((\tilde{w_k}-b^g_i)\Delta t * Exp(-J_r((\tilde{w}_k - b^g_i))\Delta t)\eta^{gd}_k\Delta t \right\} \\
= \Delta \tilde{R}_{ij} \prod^{j-1}_{k=i}Exp(-\Delta\tilde{R}^T_{k+1,j}J^k_r \eta^{gd}_k\Delta t)$$

$$Exp(-\Delta\tilde{R}^T_{k+1,j}J^k_r \eta^{gd}_k\Delta t) = Exp(-\delta\phi_{ij})$$
其中$\Delta \tilde{R}_{ij}$是旋转的预积分测量值，由陀螺仪测量值和对陀螺仪bias的估计或猜测计算得到，认为$\delta \phi_{ij}$为其测量的噪声。



推到: 
$$\prod^{j-1}_{k=i} Exp(-J_r((\tilde{w}_k)-b^g_i)\Delta t)\eta^{gd}_k\Delta t =\prod^{j-1}_{k=i}Exp(-\Delta\tilde{R}^T_{k+1,j}J^k_r\eta^{dg}_k\Delta t)$$

当$\delta \phi$是小量的时候

$$Exp(\phi + \delta \phi) \approx Exp(\phi)Exp(J_r(\phi)\delta \phi)$$

adjoint 性质

$$Exp(\phi)R = RExp(R^T\phi)$$

令 $J^k_r = J_r((\tilde{w})_k-b^g_i)\Delta t$, 再令 $\Delta \tilde{R}_{ij} = \prod^{j-1}_{k=i} Exp((\tilde{w}_k-b^g_i)\Delta t)$和$Exp(-\delta \phi_{ij})=\prod^{j-1}_{k=i}Exp(-\Delta\tilde{R}^T_{k+1,j}J^k_r\eta^{gd}_k\Delta t)$


$$\Delta v_i = \sum^{j-1}_{k=i}\Delta R_{ik} (\tilde{f_k}-b^a_i-\eta^{ad}_k)\Delta t \\ 
 \approx \sum^{j-1}_{k=i}\Delta \tilde{R}_{ik}Exp(-\delta\phi_{ik})(\tilde{f_k}-b^a_i-\eta^{ad}_k)\Delta t \\
 \approx \sum^{j-1}_{k=i}\Delta \tilde{R}_{ik}(I-\delta\phi^{\times})(\tilde{f_k}-b^a_i-\eta^{ad}_k)\Delta t \\ 
 \approx \sum^{j-1}_{k=i}\left[ \Delta\tilde{R}_{ik}(I-\delta\phi^{\times})(\tilde{f}_k-b^a_i)\Delta t - \Delta\tilde{R}_{ik}\eta^{ad}_k\Delta t\right] \\
 =\sum^{j-1}_{k=i}\left[\Delta\tilde{R}_{ik}(\tilde{f}_k - b^a_i)\Delta t+\Delta\tilde{R}_{ik}(\tilde{f}_k-b^a_i)^{\times}\delta\phi_{ik}\Delta t - \Delta\tilde{R}_{ik}\eta^{ad}_k\Delta t \right] \\
 =\sum^{j-1}_{k=i}\left[\Delta\tilde{R}_{ik}(\tilde{f}_k-b^a_i)\Delta t \right]+\sum^{j-1}_{k=i}\left[\Delta \tilde{R}_{ik}(\tilde{f}_k-b^a_i)^{\times}\phi_{ik}\Delta t- \Delta \tilde{R}_{ik}\eta^{ad}_k\Delta t \right]$$

将该公式忽略高阶小项，以及用简略的方式表示$\Delta v_{ij}$

$$\Delta \tilde{v}_{ij} = \sum^{j-1}_{k=i}\left[\Delta\tilde{R}_{ik}(\tilde{f}_k-b^a_i)\Delta t \right]$$

$$\delta v_{ij} = \sum^{j-1}_{k=i}\left[\Delta \tilde{R}_{ik}\eta^{ad}_k\Delta t- \Delta\tilde{R}_{ik}(\tilde{f}_k-b^a_i)^{\times}\delta \phi_{ik}\Delta t \right]$$

### the important
$$\Delta v_{ij} = \Delta \tilde{v}_{ij} - \delta \phi_{ij}$$



$$\Delta p_{ij} = \sum^{j-1}_{k=i}\left[\Delta v_{ik}\Delta t + \frac{1}{2}\Delta R_{ik}(\tilde{f}-b^a_i-\eta^{ad}_k)\Delta t^2\right] \\
同理带入上式子可以得到并拆分成\\
\approx\ sum^{j-1}_{k=i}\left[(\Delta\tilde{v}_{ik}-\delta v_{ik})\Delta t + \frac{1}{2}\Delta\tilde{R}_{ik}(I-\delta\phi_{ik})\Delta t^2 - \frac{1}{2}\Delta\tilde{R}_{ik}\eta^{ad}_k\Delta t^2 \right] \\
= \sum^{j-1}_{k=i}\left[\Delta \tilde{v}_{ik}+\frac{1}{2}\Delta\tilde{R}_{ik}(\tilde{f}_k-b^a_i)\Delta t^2 +\frac{1}{2}\Delta\tilde{R}_{ik}(\tilde{f}_k-b^a_i) ^{\times}\delta\phi_{ik}\Delta t^2-\frac{1}{2}\Delta\tilde{R}_{ik}\eta^{ad}_k\Delta t^2 - \delta v_{ik}\Delta t \right]$$

#### bias更新时的积分测量值更新
