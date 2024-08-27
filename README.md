# Cart-Pole System Controlled by LQR, LMPC and NMPC Controllers

## Dependencies

- Matlab (Author's version is R2021a)
- Casadi : can be installed [here](https://web.casadi.org/)

## Cart-Pole System dynamics

$$
\underbrace {\begin{bmatrix} m_c + m_p & m_pL\cos\theta \\
m_pL\cos \theta & m_pL^2\end{bmatrix}}_{\mathbf{M(q)}} \underbrace{\begin{bmatrix} \ddot x \\ \ddot \theta\end{bmatrix}}_{\mathbf{\ddot{q}}} + \underbrace{\begin{bmatrix} 0 & -m_pL\sin\theta \dot \theta \\ 0 & 0\end{bmatrix}}_{\mathbf{C(q,\dot{q})}}  \underbrace{\begin{bmatrix} \dot x \\ \dot \theta\end{bmatrix}}_{\mathbf{\dot{q}}} + \underbrace{\begin{bmatrix} 0 \\ -m_pgL\sin \theta\end{bmatrix}} _{\mathbf{g(q)}} = \underbrace{\begin{bmatrix} f \\ 0\end{bmatrix}}_{\boldsymbol \tau}
$$

<img src="cart_pole_dynamics.png" alt="dynamics" style="zoom: 40%;" />

## Results

NMPC:

![anim](cartpole.gif)
