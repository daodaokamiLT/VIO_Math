## Online Calibration for cam and imu

1. temporal offset
$$t_{imu} = t_{cam}+t_d$$

2. Feature Velocity on Image Plane



<!-- estimate the depth can use the points speed, the rotation velocity has a proj to the 2dpoints speed reference just with the distance -->

we specifically shift features' observations in the timeline. to this end, introduce feature velocity for modeling and compensating the temporal misalignment.

we assume the features in plane is a constant velocity. so we compute the features' velocity on the image plane.

feature l also moves at a constant speed $V^k_l$
$$V^k_l = \frac{\left(\left[\begin{matrix}u^{k+1}_l \\ v^{k+1}_l \end{matrix}\right]-\left[\begin{matrix}
    u^k_l \\ v^k_l
\end{matrix} \right] \right)}{(t_{k+1} - t_k)}$$

change to the speed.

3. vision factor with time offset
the reprojection error add $t_d$.

the ori error function can be respected as:
$$e^k_l = z^k_l - \pi(R^{w^T}_{c_k}(P_l - p^w_{c_k}))$$

$$z^k_l = \left[ \begin{matrix}
    u^k_l & v^k_l 
\end{matrix}\right]^T$$

change to:
$$e^k_l = z^k_l(t_d) - \pi(R^{w^T}_{c_k}(P_l - p^w_{c_k}))$$

$$z^k_l(t_d) = \left[\begin{matrix}
    u^k_l & v^k_l
\end{matrix} \right]^T + t_dV^k_l$$

also has Depth parameterization.

$\lambda_i$ is depth of i point in image. projection error function can be writen as:

$$e^j_l = z^j_l - \pi(R^{w^T}_{c_j}(R^w_{c_i}\lambda_i\left[\begin{matrix}
    z^i_l \\ 1
\end{matrix}\right]+p^w_{c_i}-p^w_{c_j}))$$

$$z^i_l = \left[\begin{matrix} u^i_l & v^i_l \end{matrix}\right]^T, z^j_l = \left[\begin{matrix} u^j_l & v^j_l \end{matrix}\right]^T$$

and make $t_d$ into the costfunction.

$$e^j_l = z^j_l(t_d) - \pi(R^{w^T}_{c_j}(R^w_{c_i}\lambda_i\left[\begin{matrix}
    z^i_l(t_d) \\ 1
\end{matrix}\right]+p^w_{c_i}-p^w_{c_j}))$$

$$z^i_l = \left[\begin{matrix} u^i_l & v^i_l \end{matrix}\right]^T + t_d V^i_l, z^j_l = \left[\begin{matrix} u^j_l & v^j_l \end{matrix}\right]^T+t_dV^j_l$$

4. than optimization with Time offset
status is $X=\left[x_0, x_1, \cdots,x_n,P_0,P_1,\cdots,P_l, t_d \right]$, the $x_k$ is $x_k = [p^w_k, v^w_k, R^w_k, b_a, b_g], k \in [0, n]$

whole problem is formulated as one cost function containing IMU propagation factor, reprojection factor as well as a certain prior factor. So the paper use the next factor to achieve time offset calibration.
depart is prior factor + imu propagation factor + proposd vision factor.
$$\min_x\left\{\left|e_p-H_pX \right| ^2 + \sum_{k\in B}\left|e_B(z^k_{k+1}, X) \right|^2_{P^k_{k+1}}+\sum_{(l,j)\in C}\left|e_C(z^j_l, X) \right|^2_{P^j_l}\right\}$$



5. Compensation of time offset.