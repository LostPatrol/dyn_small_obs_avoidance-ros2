<!-- Core mathematics for constant-acceleration primitive clearance certification. -->
# 恒加速度原语的自适应净空判定

$\mathcal O$ 为固定障碍点集，$r$ 为最小净空，$I=[t_l,t_r]$，$h=t_r-t_l$。

## 运动方程

$$
p(t)=p_0+v_0t+\frac12at^2,\qquad v(t)=v_0+at.
$$

$$
v((1-\lambda)t_l+\lambda t_r)
=(1-\lambda)v(t_l)+\lambda v(t_r),\qquad 0\le\lambda\le1.
$$

## 障碍距离

$$
d(x)=\min_{q\in\mathcal O}\|x-q\|_2,\qquad
|d(x)-d(y)|\le\|x-y\|_2.
$$

$$
D_l=d(p(t_l)),\qquad D_r=d(p(t_r)).
$$

## 弧长上界

$$
\|v((1-\lambda)t_l+\lambda t_r)\|_2
\le(1-\lambda)\|v(t_l)\|_2+\lambda\|v(t_r)\|_2.
$$

$$
L=\int_{t_l}^{t_r}\|v(t)\|_2\,dt
\le\widehat L=\frac h2\bigl(\|v(t_l)\|_2+\|v(t_r)\|_2\bigr).
$$

## 连续净空判定

$$
\forall t\in I:\quad
d(p(t))\ge\frac{D_l+D_r-L}{2}
\ge\frac{D_l+D_r-\widehat L}{2}.
$$

$$
\boxed{
D_l\ge r,\quad D_r\ge r,\quad
D_l+D_r\ge2r+\widehat L
\ \Longrightarrow\
\min_{t\in I}d(p(t))\ge r
}
$$

## 自适应二分

$$
t_m=\frac{t_l+t_r}{2},\qquad
D_m=d(p(t_m)),\qquad
I_l=[t_l,t_m],\quad I_r=[t_m,t_r].
$$

$$
\operatorname{Cert}(I)=
\begin{cases}
\mathrm{false}, & \min(D_l,D_r)<r,\\
\mathrm{true}, & D_l+D_r\ge2r+\widehat L,\\
\operatorname{Cert}(I_l)\land\operatorname{Cert}(I_r), & \text{否则}.
\end{cases}
$$

$$
\text{有限细分后仍未证明安全}\ \Longrightarrow\ \text{拒绝该原语}.
$$
