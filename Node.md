1. QP 数值病态（HQPID 独有）
现象：QP 超时/MaxIterReached，不收敛
根因：动力学约束 M*qddot - tau = -nle 中 M 跨 5 个数量级（500→0.005），KKT 系统严重病态
修复：对动力学约束行缩放——每行除以 √M_ii，约束行范数从 500:1 降到 1.6:1
2. QP 滥用重型关节（两类 HQP 都受影响）
现象：j1（500kg 棱柱关节）力矩撞 ±30000 极限，QP 力矩比解析解大 8 倍
根因：QP 的 ||qdot/qddot||² 惩罚对所有关节相等，但重型关节需要巨大力矩
修复：阻尼权重与质量矩阵对角线成正比（damping_i = M_ii * scale）。QP 自然避开重型关节
3. 旋转漂移（长时间运行累积）
现象：反复平移后夹爪翘起
根因：旋转和位移 QP 跟踪权重相同，QP 牺牲旋转来减少重型关节使用
修复：旋转跟踪权重 3 倍于位移，旋转 Kp 更高

但我要诚实地告诉你：HQPIK 对 500kg 臂有本质架构限制。

HQPIK 的两层架构（运动学 QP → 关节 PD）像"一个人看地图指挥，另一个人开车"——看地图的不知道车有多重，指挥的速度（qdot_des）司机（PD）跟不上。

                 HQPIK                HQPID
架构	         QP(运动学) → PD(动力学)	QP(运动学+动力学一体)
知道惯性吗	      ❌ QP 不知道         	✅ 动力学约束中有 M
500kg 臂适用	 ⚠️ 外环必须极慢     	✅ 完美匹配
当前状态	      Kp=1 勉强工作         	已稳定运行
建议：任务空间控制用 HQPID（ur8_hqpid），它已经调好了。HQPIK 保留给不需要动力学的场景（如纯运动学规划）。

QPID + 导纳控制 = 在位置控硬件上获得力矩控的好处——这可能是你最终要的方案。



键盘输入
    ↓
┌─────────────────────────────────────────────────────┐
│ 第 1 层：Cubic 轨迹规划（平滑插值）                    │
│ x(t) = a₀ + a₁t + a₂t² + a₃t³                       │
│ C² 连续：位置、速度、加速度都连续                       │
└─────────────────────────────────────────────────────┘
    ↓  x_des(t), ẋ_des(t)
┌─────────────────────────────────────────────────────┐
│ 第 2 层：Task-Space PD（误差→期望加速度）              │
│ ẍ_des = K_p · (x_des − x) + K_v · (ẋ_des − ẋ)       │
└─────────────────────────────────────────────────────┘
    ↓  ẍ_des
┌─────────────────────────────────────────────────────┐
│ 第 3 层：HQP QP 优化（加速度→力矩）                    │
│                                                     │
│  成本函数（最小化）：                                   │
│    ‖Jq̈ + J̇q̇ − ẍ_des‖²_W_track                       │
│    + ‖q̈‖²_W_acc + ‖Δt·q̈ + q̇‖²_W_vel + 100·Σ slack  │
│                                                     │
│  等式约束：Mq̈ − τ = −nle   （刚体动力学）               │
│                                                     │
│  不等式约束 (CBF)：                                    │
│    关节限位、速度限位、自碰撞、奇异点                    │
│                                                     │
│  边界约束：τ_min ≤ τ ≤ τ_max                          │
└─────────────────────────────────────────────────────┘
    ↓  τ
  机械臂

第 3 层完整数学展开
决策变量（共 $6n+2 = 62$ 个，n=10 关节）：

$$
\begin{bmatrix} \ddot{q} \ \tau \ s_{q_{min}} \ s_{q_{max}} \ s_{\dot{q}{min}} \ s{\dot{q}{max}} \ s{sing} \ s_{col} \end{bmatrix}
\quad
\begin{aligned}
\ddot{q} &\in \mathbb{R}^{10} &\text{关节加速度} \
\tau &\in \mathbb{R}^{10} &\text{关节力矩} \
s_* &\in \mathbb{R}_+ &\text{松弛变量}
\end{aligned}
$$

成本函数（二次型）：

$$
\min \quad \underbrace{\sum_i |J_i\ddot{q} + \dot{J}i\dot{q} - \ddot{x}{des,i}|^2_{W_{track}}}_{\text{任务跟踪}}

\underbrace{|\ddot{q}|^2_{W_{acc}}}_{\text{加速度阻尼}}
\underbrace{|\Delta t \cdot \ddot{q} + \dot{q}|^2_{W_{vel}}}_{\text{速度阻尼}}
\underbrace{100 \cdot \sum s}_{\text{松弛惩罚}} $$
等式约束（行缩放后）：

行缩放矩阵 $S_{ii} = \frac{1}{\sqrt{M_{ii}}}$：

$$
\begin{bmatrix} S \cdot M & -S \end{bmatrix}
\begin{bmatrix} \ddot{q} \ \tau \end{bmatrix}
= -S \cdot nle
$$

即 $S_i(M_i\ddot{q} - \tau_i) = -S_i \cdot nle_i$，约束每行范数均衡（1422 而非 ~500）。

不等式约束 — 关节限位 CBF（$\alpha=100$）：

二阶 CBF：$h(q) = q - q_{min} \geq 0$

对时间求导两次得约束不等式：

$$
\ddot{q} \geq -2\alpha \cdot \dot{q} - \alpha^2(q - q_{min}) + s_{q_{min}}
$$

不等式约束 — 速度限位 CBF：

一阶 CBF：$h(\dot{q}) = \dot{q}_{max} - \dot{q} \geq 0$

$$
-\ddot{q} \geq -\alpha(\dot{q}{max} - \dot{q}) + s{\dot{q}_{max}}
$$

不等式约束 — 自碰撞 CBF：

最短距离 $d(q)$ 及其梯度、梯度导数经低通滤波后：

$$
\nabla d \cdot \ddot{q} \geq -\dot{\nabla d} \cdot \dot{q} - 2\alpha \cdot \nabla d \cdot \dot{q} - \alpha^2(d - 0.01) + s_{col}
$$

不等式约束 — 奇异点 CBF：

可操作性 $m = \sqrt{\det(JJ^T)}$，$m_{min}=0.02$：

$$
\nabla m \cdot \ddot{q} \geq -\dot{\nabla m} \cdot \dot{q} - 2\alpha \cdot \nabla m \cdot \dot{q} - \alpha^2(m - m_{min}) + s_{sing}
$$

边界约束：

$$
\tau_{min} \leq \tau \leq \tau_{max}, \quad s_* \geq 0
$$

层级结构的含义
当前只有一个层级（EE 跟踪）。如果添加 L2（如姿态优化）、L3（如避障），更高层的等式约束会把低层的 Jacobian 堆叠为等式约束——低层解不被高层改变：


L1(EE跟踪)求解 → 得到 q̈₁, τ₁
    ↓ J₁ 堆叠为等式约束
L2(姿态)求解  → J₁·q̈ = J₁·q̈₁ （保留 L1 的 EE 跟踪）
    ↓ 在满足 L1 的前提下优化姿态
    ↓ J₁+J₂ 堆叠
L3(避障)求解  → 在满足 L1+L2 的前提下避障

这就是 HQP（层级 QP）的核心：高优先级任务的解成为低优先级任务的硬约束。

各层数学
L1 — EE 跟踪：

$$
J_{ee} \cdot \ddot{q} = \ddot{x}{des} - \dot{J}{ee} \cdot \dot{q}
$$

6 个等式约束。QP 解出的 $\ddot{q}_1$ 满足此约束。

L2 — 关节姿态：

在不破坏 L1 的前提下（即 $J_{ee} \cdot \ddot{q} = J_{ee} \cdot \ddot{q}1$），尽量使关节趋近舒适位姿 $q{pref}$：

$$
\min_{\ddot{q}} \quad |\ddot{q} + 2\zeta\omega_n\dot{q} + \omega_n^2(q - q_{pref})|^2
$$

$$
\text{s.t.} \quad J_{ee} \cdot \ddot{q} = J_{ee} \cdot \ddot{q}_1 \quad \text{(L1 的等式约束被堆叠)}
$$

实现方式：HQPID 自动把 L1 的 $J_{ee}$ 堆叠为 L2 的等式约束矩阵：

$$
\begin{bmatrix} J_{ee} \ J_{posture} \end{bmatrix} \ddot{q} = \begin{bmatrix} \ddot{x}{des} - \dot{J}{ee}\dot{q} \ \ddot{q}_{posture,des} \end{bmatrix}
$$

L3 — 操作度最大化：

和 L2 同样的机制——L1+L2 的 Jacobian 都堆叠为 L3 的等式约束。在零空间内最大化 $m$：

$$
\min_{\ddot{q}} \quad |-\nabla m \cdot \ddot{q}|^2 \quad \text{s.t. L1, L2 都满足}
$$