# Explicação completa da versão final (V1) — contas e código

Este documento segue o caminho dos dados do princípio ao fim: **da tabela D-H
até ao robô a mexer-se e aos erros medidos**. Em cada passo diz *o quê*, *as
contas* e *onde está no código* (os links abrem o ficheiro na linha certa no
GitHub/VS Code). Para o contexto geral e o histórico das decisões, ver o
[GUIA.md](GUIA.md).

---

## 0. Mapa: o caminho dos dados

```
   config.py            kinematics.py              trajectory.py                     run_robot.py / ik_compare.py
 ─────────────      ──────────────────────      ─────────────────────────────      ───────────────────────────────
 limites,           (1) FK D-H: q -> T         (3) ângulos do swing φ(t), ψ(t)      (7) poses para o TCP do robô
 pinça 83.6 mm      (2) R <-> roll,pitch,yaw   (4) ponta da pinça p(t), R(t)        (8) enviar: aproximação + servo
                    (5) IK numérica            (4b) flange = ponta - 83.6·z          (9) Tarefa 4: IK do robô -> FK
                                               (6) verificações (velocidades,
                                                   limites, mesa, base)
```

| Passo | O quê | Ficheiro e função |
|---|---|---|
| 1 | Cinemática direta (Tarefa 1) | [kinematics.py:61 `link_transform`](src/kinematics.py#L61), [kinematics.py:73 `fk`](src/kinematics.py#L73) |
| 1b | Validação com a tabela D-H clássica | [kinematics.py:84 `fk_standard`](src/kinematics.py#L84), [kinematics.py:100 `validate`](src/kinematics.py#L100) |
| 2 | Orientação ↔ roll/pitch/yaw | [kinematics.py:112 `rpy_from_R`](src/kinematics.py#L112), [kinematics.py:120 `R_from_rpy`](src/kinematics.py#L120) |
| 3 | Perfis temporais do swing | [trajectory.py:85 `swing_angles`](src/trajectory.py#L85) |
| 4 | Posição e orientação da ponta da pinça (Tarefa 2) | [trajectory.py:114 `swing_trajectory`](src/trajectory.py#L114) |
| 5 | Ângulos das juntas (IK numérica, só para verificar) | [kinematics.py:179 `ik_numeric`](src/kinematics.py#L179), [trajectory.py:156 `joint_trajectory`](src/trajectory.py#L156) |
| 6 | Viabilidade | [trajectory.py:169 `check_feasibility`](src/trajectory.py#L169) |
| 7 | Conversão para o TCP do controlador | [trajectory.py:139 `command_poses`](src/trajectory.py#L139), [robot.py:26 `controller_tcp`](src/robot.py#L26) |
| 8 | Execução no robô (Tarefa 3) | [run_robot.py:89 `main`](src/run_robot.py#L89), [run_robot.py:56 `run_servo`](src/run_robot.py#L56), [run_robot.py:47 `run_pontos`](src/run_robot.py#L47) |
| 9 | Precisão da IK do robô (Tarefa 4) | [ik_compare.py:38 `collect`](src/ik_compare.py#L38), [ik_compare.py:63 `analyse`](src/ik_compare.py#L63) |

Convenções em todo o código: **mm e graus**, referencial da **base** do robô
(z para cima), orientação em roll/pitch/yaw com $R = R_z(\text{yaw})\,R_y(\text{pitch})\,R_x(\text{roll})$ — as mesmas do SDK.

---

## 1. Cinemática direta (Tarefa 1)

### 1.1 Método: D-H modificado (Craig)

Cada junta $i$ tem um referencial $\{i\}$ com o eixo $z_i$ no eixo de rotação.
A passagem de $\{i-1\}$ para $\{i\}$ são 4 movimentos elementares:

$$
{}^{i-1}T_i = R_x(\alpha_{i-1})\;T_x(a_{i-1})\;R_z(\theta_i)\;T_z(d_i),\qquad \theta_i = q_i + \text{offset}_i
$$

Multiplicando as quatro matrizes 4×4 obtém-se a matriz de cada elo
(é exatamente o que está escrito em [kinematics.py:61](src/kinematics.py#L61)):

$$
{}^{i-1}T_i=\begin{bmatrix}
\cos\theta & -\sin\theta & 0 & a\\
\sin\theta\cos\alpha & \cos\theta\cos\alpha & -\sin\alpha & -d\sin\alpha\\
\sin\theta\sin\alpha & \cos\theta\sin\alpha & \cos\alpha & d\cos\alpha\\
0&0&0&1\end{bmatrix}
$$

e a pose do flange é o produto dos seis ([kinematics.py:73](src/kinematics.py#L73), o ciclo `T = T @ link_transform(...)`):

$$
{}^{0}T_6(q) = {}^{0}T_1\,{}^{1}T_2\,{}^{2}T_3\,{}^{3}T_4\,{}^{4}T_5\,{}^{5}T_6
=\begin{bmatrix} R & p\\ 0 & 1\end{bmatrix}
$$

$R$ (3×3) é a orientação do flange e $p$ (mm) a posição.

### 1.2 A tabela (fonte: PDF oficial, pág. 1) — [kinematics.py:40](src/kinematics.py#L40)

| Junta | offset θ | d [mm] | α_{i−1} | a_{i−1} [mm] |
|---|---|---|---|---|
| J1 | 0° | 243.3 | 0° | 0 |
| J2 | −90° | 0 | −90° | 0 |
| J3 | −90° | 0 | 180° | 200 |
| J4 | 0° | 227.6 | 90° | 87 |
| J5 | 0° | 0 | 90° | 0 |
| J6 | 0° | 61.5 | −90° | 0 |

243.3 = altura do ombro; 200 = braço; 87 = desvio do antebraço; 227.6 =
antebraço; 61.5 = pulso → flange. J4, J5, J6 formam um **pulso esférico**.

### 1.3 Conta de verificação: q = 0

Com $q = 0$ o braço sobe 200 mm, o antebraço avança 87 e desce 227.6, o pulso
desce 61.5:

$$p = (87,\; 0,\; 243.3 + 200 - 227.6 - 61.5) = (87,\;0,\;154.2)\ \text{mm},\quad \text{roll} = 180°$$

`python src/kinematics.py` dá exatamente isto.

### 1.4 Validação forte — [kinematics.py:84](src/kinematics.py#L84), [kinematics.py:100](src/kinematics.py#L100)

O mesmo PDF dá a tabela em **D-H clássico** (outros referenciais, outra fórmula:
${}^{i-1}T_i = R_z(\theta_i)\,T_z(d_i)\,T_x(a_i)\,R_x(\alpha_i)$). Implementada à
parte, comparada em 10 000 $q$ aleatórios: diferença máxima **2.3·10⁻¹³ mm**
(arredondamento). `python src/kinematics.py --validar`.

**No robô real** (lab 5/11): nossa FK vs FK do controlador = **1.5 mm em média,
2.1 mm no máximo** — o controlador usa a calibração de fábrica deste robô.

---

## 2. Orientação: matriz ↔ roll/pitch/yaw — [kinematics.py:112](src/kinematics.py#L112)

Com $R = R_z(y)\,R_y(p)\,R_x(r)$, os elementos da matriz dão:

$$
\text{pitch} = \arcsin(-R_{20}),\qquad
\text{roll} = \operatorname{atan2}(R_{21}, R_{22}),\qquad
\text{yaw} = \operatorname{atan2}(R_{10}, R_{00})
$$

(índices a começar em 0). A volta, $R(r,p,y)$, é o produto das três rotações
elementares ([kinematics.py:120](src/kinematics.py#L120)).

Para medir **diferenças** de orientação usa-se o **logaritmo** de SO(3)
([kinematics.py:159 `rotvec`](src/kinematics.py#L159)): para uma rotação $R$,

$$
\theta = \arccos\frac{\operatorname{tr}R - 1}{2},\qquad
\hat k = \frac{1}{2\sin\theta}\begin{bmatrix}R_{21}-R_{12}\\R_{02}-R_{20}\\R_{10}-R_{01}\end{bmatrix}
$$

e o erro entre $R_a$ e $R_b$ é o ângulo $\theta$ de $R_a R_b^T$
([kinematics.py:174](src/kinematics.py#L174)) — um só número, sem problemas de
convenção.

---

## 3. O tempo: perfis do swing — [trajectory.py:85 `swing_angles`](src/trajectory.py#L85)

O swing é descrito por dois ângulos, como o pêndulo duplo do Lab 1:
- $\varphi(t)$ — o **"braço"**: onde a ponta da pinça está no arco ($\varphi = 0$ na bola);
- $\psi(t)$ — o **"pulso"**: quanto a pinça roda a mais (armada no topo, solta tarde).

Valores ([trajectory.py:48 `DEFAULTS`](src/trajectory.py#L48)):
$\varphi_{top} = -90°$, $\varphi_{end} = 50°$, $\psi_{top} = 30°$,
$T_b = 2.0$ s, $T_d = 2.2$ s, $f_{release} = 0.6$, $dt = 0.01$ s.

### 3.1 Backswing ($0 \le t \le T_b$) — [trajectory.py:98](src/trajectory.py#L98)

Perfil **quíntico** $s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5$, com $\tau = t/T_b$:

$$\varphi(t) = \varphi_{top}\,s(\tau),\qquad \psi(t) = \psi_{top}\,s(\tau)$$

Porquê grau 5: $s(0)=0,\ s(1)=1$ e $s'$ e $s''$ são **zero nos dois extremos** —
parte e chega parado e sem solavanco. A velocidade máxima é a meio:
$s'(\tau) = 30\tau^2 - 60\tau^3 + 30\tau^4$, $s'(0.5) = 30/16 = 1.875$, logo

$$|\dot\varphi|_{max} = 1.875\,\frac{|\varphi_{top}|}{T_b} = 1.875 \times \frac{1.571}{2.0} = 1.47\ \text{rad/s}
\;\Rightarrow\; v = 1.47 \times 150 = \mathbf{221\ mm/s}$$

(É **este** o máximo de velocidade da ponta da pinça em todo o swing — o
"vel. TCP 220.9 mm/s" da verificação.)

### 3.2 Downswing ($T_b < t \le T_b + T_d$) — [trajectory.py:103](src/trajectory.py#L103)

Com $u = t - T_b$, **aceleração constante** (igual ao Lab 1):

$$\varphi(u) = \varphi_{top}\Bigl(1 - \bigl(\tfrac{u}{T_d}\bigr)^2\Bigr)
\quad\Rightarrow\quad
\dot\varphi(u) = -\frac{2\varphi_{top}\,u}{T_d^2},\qquad \ddot\varphi = -\frac{2\varphi_{top}}{T_d^2}\ (\text{constante})$$

- $u = 0$: $\varphi = \varphi_{top}$, $\dot\varphi = 0$ → encaixa com o fim do backswing;
- $u = T_d$: $\varphi = 0$ (**bola**) com velocidade máxima
  $\omega_{imp} = -2\varphi_{top}/T_d = 2 \times 1.571 / 2.2 = 1.43$ rad/s
  ([trajectory.py:91](src/trajectory.py#L91)) → $v_{imp} = 1.43 \times 150 = 214$ mm/s.

Aceleração: $2 \times 1.571 / 2.2^2 = 0.65$ rad/s². Aceleração constante é o que
dá mais velocidade na bola para a mesma aceleração máxima.

**Pulso** ([trajectory.py:105](src/trajectory.py#L105)): fica armado até
$t_{rel} = f_{release}\,T_d = 1.32$ s e depois solta com um quíntico até 0
exatamente em $u = T_d$:

$$\psi(u) = \psi_{top}\Bigl(1 - s\bigl(\tfrac{u - t_{rel}}{T_d - t_{rel}}\bigr)\Bigr)$$

### 3.3 Follow-through ($t > T_b + T_d$) — [trajectory.py:110](src/trajectory.py#L110)

Desaceleração constante $a$ desde $\omega_{imp}$ até parar em $\varphi_{end}$.
Com velocidade inicial $\omega$ e final 0 em tempo $T_f$:
$\varphi_{end} = \omega T_f / 2$, logo ([trajectory.py:92](src/trajectory.py#L92))

$$T_f = \frac{2\varphi_{end}}{\omega_{imp}} = \frac{2 \times 0.873}{1.43} = 1.22\ \text{s},\qquad
a = \frac{\omega_{imp}}{T_f} = 1.17\ \text{rad/s}^2,\qquad
\varphi(u) = \omega_{imp}\,u - \tfrac12 a u^2$$

**Duração total:** $2.0 + 2.2 + 1.22 = 5.42$ s → 543 pontos a 100 Hz.
Com `--lento N`, $T_b$ e $T_d$ multiplicam por N (e $T_f$ também, porque
$\omega_{imp}$ divide por N): a `--lento 3` são 16.3 s.

---

## 4. O espaço: posição e orientação da pinça (Tarefa 2) — [trajectory.py:114](src/trajectory.py#L114)

### 4.1 Posição: um arco

Bola $b = (190, 0, 80)$ mm, direção do alvo $\hat d = (0,1,0)$ (o swing é
**para o lado**), raio $R_a = 150$ mm. O "ombro" virtual fica por cima da bola:
$c = b + R_a\,\hat z = (190, 0, 230)$. A ponta da pinça ([trajectory.py:129](src/trajectory.py#L129)):

$$p(t) = c + R_a\bigl(\sin\varphi(t)\,\hat d - \cos\varphi(t)\,\hat z\bigr)
= \bigl(190,\; 150\sin\varphi,\; 230 - 150\cos\varphi\bigr)$$

| fase | φ | p [mm] |
|---|---|---|
| bola (início e impacto) | 0° | (190, 0, 80) |
| topo | −90° | (190, −150, 230) |
| fim | 50° | (190, 115, 134) |

### 4.2 Orientação: rotação em torno da normal ao plano

Normal ao plano do swing: $\hat n = \hat d \times \hat z = (0,1,0)\times(0,0,1) = (1,0,0)$.
Orientação na bola: $R_0 = R(\text{roll}=180°, \text{pitch}=45°, \text{yaw}=0)$ — a
pinça para baixo e **inclinada 45°** (a pique, a J5 ficava em 0 = singularidade,
§5.3). Durante o swing ([trajectory.py:132](src/trajectory.py#L132)):

$$R(t) = \operatorname{Rot}\bigl(-\hat n,\ \varphi(t) + \psi(t)\bigr)\,R_0$$

com a fórmula de **Rodrigues** ([trajectory.py:78](src/trajectory.py#L78)),
para eixo unitário $\hat k$ e ângulo $\theta$, $K$ = matriz do produto externo por $\hat k$:

$$\operatorname{Rot}(\hat k,\theta) = I + \sin\theta\,K + (1-\cos\theta)\,K^2,
\qquad K = \begin{bmatrix}0&-k_z&k_y\\k_z&0&-k_x\\-k_y&k_x&0\end{bmatrix}$$

$p(t)$ e $R(t)$ são as **"funções do tempo da posição e orientação da pinça"**
que o enunciado pede.

### 4.3 Da ponta ao flange: a pinça — [trajectory.py:134](src/trajectory.py#L134)

A pinça (Gripper Lite) tem o TCP a $L = 83.6$ mm do flange ao longo do $z$ da
ferramenta (URDF oficial). O $z$ da ferramenta é a 3.ª coluna de $R$:

$$p_{flange}(t) = p(t) - L\,R(t)\,\hat e_z$$

Na bola: $R_0\hat e_z = (-\sin45°, 0, -\cos45°) = (-0.707, 0, -0.707)$, logo
$p_{flange} = (190 + 59.1,\ 0,\ 80 + 59.1) = (249.1, 0, 139.1)$ — a "primeira pose
enviada" que se viu no robô.

---

## 5. Os ângulos das juntas: IK numérica — [kinematics.py:179](src/kinematics.py#L179)

Para verificar limites e velocidades **das juntas** é preciso $q(t)$. A IK do
robô só existe ligada ao robô, por isso há uma IK própria (não substitui a da
Tarefa 4).

### 5.1 Mínimos quadrados amortecidos (Levenberg–Marquardt)

Erro entre a pose desejada $(p^*, R^*)$ e a atual $(p(q), R(q))$:

$$e(q) = \begin{bmatrix} p^* - p(q) \\ w\cdot\log\bigl(R^* R(q)^T\bigr)^\vee\end{bmatrix}\in\mathbb R^6,\qquad w = 100\ \text{mm/rad}$$

Jacobiano por diferenças finitas ([kinematics.py:204](src/kinematics.py#L204)), $h = 10^{-4}$°:

$$J_{:,j} \approx -\frac{e(q + h\,\hat e_j) - e(q)}{h}$$

Passo ([kinematics.py:205](src/kinematics.py#L205)):

$$\Delta q = J^T\bigl(JJ^T + \lambda^2 I\bigr)^{-1} e,\qquad \lambda = 1$$

$\lambda^2 I$ impede passos gigantes quando $J$ é quase singular. Repete-se
(cortando $q$ aos limites) até $\|e_p\| < 10^{-3}$ mm e $\|e_R\| < 10^{-5}$ rad.

### 5.2 Ao longo da trajetória — [trajectory.py:156](src/trajectory.py#L156)

A IK resolve o **flange** em cada um dos 543 pontos, começando na solução do
ponto anterior → as juntas variam de forma contínua e no mesmo "ramo"
(cotovelo/pulso).

### 5.3 A singularidade do pulso (porque a pinça está a 45°)

Com a pinça a pique, $q_5 = 0$ e os eixos de J4 e J6 alinham: o jacobiano perde
característica. Para um pulso esférico, o bloco do pulso tem determinante
proporcional a $\sin q_5$, e as velocidades das juntas são $\dot q = J^{-1}\nu$,
com $J^{-1} \propto 1/\sin q_5$ → quando $q_5 \to 0$, J4 e J6 disparam (~1900 °/s
no primeiro teste). Com a pinça a 45°, J5 fica entre 61° e 116°.

---

## 6. Viabilidade — [trajectory.py:169 `check_feasibility`](src/trajectory.py#L169)

Velocidades por **diferenças centrais** (`np.gradient`):
$\dot x_k \approx (x_{k+1} - x_{k-1}) / (2\,dt)$.

| Verificação | Conta | Limite | Valor (--lento 1) | Enunciado |
|---|---|---|---|---|
| vel. ponta da pinça | $\max_k\lVert\dot p_k\rVert$ | 0.8 × 500 = 400 mm/s | 221 | (i) |
| vel. flange | $\max_k\lVert\dot p_{flange,k}\rVert$ | 400 mm/s | 298 | (i) |
| vel. juntas | $\max_{k,i}\lvert\dot q_{i,k}\rvert$ | 0.8 × 180 = 144 °/s | 112 | (i) |
| alcance | $\max_k\lVert p_{flange,k} - p_{\{2\}}\rVert$ | 0.8 × 440 = 352 mm | 321 | (ii) |
| limites das juntas | $q_{min}+5° \le q_i \le q_{max}-5°$ | | folga mín. 7.8° (J5) | (iii) |
| acima da mesa | z da ponta, do flange e de todas as juntas | ≥ 30 mm | 80 | contacto |
| fora da base | distância ao eixo de J1 da ponta e do flange | ≥ 80 mm | 190 | contacto |

Limites do fabricante (180 °/s, 500 mm/s) confirmados nas especificações
oficiais ([config.py](src/config.py)). **Aviso** (não chumba): o flange passa
dos ~220 mm/s que o controlador do lab mostrou aguentar em servo cartesiano
(`CART_SPEED_OBSERVED`), por isso a `--lento 1` o robô fica para trás.

---

## 7. Do plano ao robô: que pose se envia — [trajectory.py:139](src/trajectory.py#L139)

O SDK move o ponto que o **controlador** considera a ferramenta (`tcp_offset`,
lido em [robot.py:26](src/robot.py#L26)). Com transformações homogéneas:

$$T_{ponta} = T_{flange}\,T_g,\qquad T_g = \operatorname{Trans}(0,0,83.6)$$

e o controlador espera a pose de $T_{flange}\,T_c$ ($T_c$ = o seu offset):

$$T_{enviar} = T_{ponta}\;T_g^{-1}\;T_c$$

No lab $T_c = I$ (offset zero) → envia-se o flange. Testado com três offsets
diferentes: a ponta cai sempre na bola.

---

## 8. Execução no robô (Tarefa 3) — [run_robot.py](src/run_robot.py)

1. **Verificação** (§6): se falhar, não envia nada ([run_robot.py:89](src/run_robot.py#L89)).
2. **Ligar** e ler `tcp_offset`; converter as poses (§7); mostrar a 1.ª pose;
   pedir `sim`.
3. **Aproximação junta a junta** a 20 °/s até $q_0$ (os ângulos da IK do ponto
   inicial) com `set_servo_angle` ([run_robot.py:134](src/run_robot.py#L134)) —
   a partir de uma pose qualquer, ir em linha reta no espaço podia passar por
   sítios estranhos. Depois um pequeno `set_position` até à pose exata.
4. **Swing:**
   - `servo` ([run_robot.py:56](src/run_robot.py#L56)): modo 1; no ponto $k$
     envia `set_servo_cartesian`, lê `get_position`, e espera ativamente até
     $t_0 + (k+1)\,dt$. Grava referência e medido em
     `data/run_servo_lento<N>_phi<fim>.csv` (nunca por cima; grava mesmo se parar);
   - `pontos` ([run_robot.py:47](src/run_robot.py#L47)): modo 0, 1 ponto em cada
     10 com `set_position(radius=5)` (blending) e a velocidade média de cada troço.
     Só com `--lento ≥ 2`.

### 8.1 O que se mediu (5/11) e a correção do C31

| Ensaio | Resultado |
|---|---|
| servo, φ_end 70°, lento 2 | **C31** sempre em φ ≈ 69.4° (ponto 1151–1152/1183) |
| pontos, lento 1 | **C24** (velocidade) |
| servo, **φ_end 50°**, lento 1 e 1.5 | **completos**, sem erros |

- **C31** = deteção de colisão (corrente anormal). Num ponto fixo e quase parado
  (10 mm/s), com o braço esticado para o lado (J5 = 112°) — não é dinâmica.
  Hipóteses: cabo da pinça a esticar, ou massa da pinça não configurada no
  controlador. **Correção:** $\varphi_{end} = 50°$ → pose final com J5 = 92°.
- **C24** → `pontos` recusa `--lento < 2`.
- **Seguimento** (erro $\lVert p_{medido} - p_{pedido}\rVert$):

| --lento | flange pedido (máx) | erro médio / máx |
|---|---|---|
| 2 (φ 70°) | 149 mm/s | 2.0 / 14.7 mm (1.1 mm descontando ~10 ms de atraso) |
| 1.5 | 198 mm/s | 5.8 / 29.1 mm |
| 1 | 298 mm/s | 18.9 / 58.8 mm (atraso ~90 ms: o controlador não passa de ~220 mm/s) |

---

## 9. Precisão da IK do robô (Tarefa 4) — [ik_compare.py](src/ik_compare.py)

Para 60 pontos $k$ da trajetória ([ik_compare.py:38](src/ik_compare.py#L38)):

1. $q_k$ = `arm.get_inverse_kinematics(pose_k)` — **IK do robô**;
2. $\hat T_k = {}^0T_6(q_k)\,T_c$ — **a nossa FK** (§1), composta com o offset do controlador;
3. erros ([ik_compare.py:67](src/ik_compare.py#L67)):
   $e_p = \lVert \hat p_k - p_k\rVert$, $e_R = \angle(\hat R_k R_k^T)$ (§2).

Também `arm.get_forward_kinematics(q_k)` (FK do robô).

**Resultados (lab):**
- FK do robô ∘ IK do robô vs pedido: **≤ 0.009 mm, ≤ 0.001°** → a IK do robô é
  exata face ao seu próprio modelo; 60/60 pontos resolvidos;
- nossa FK ∘ IK do robô vs pedido: **1.48 mm médio, 2.10 mm máx., 0.06°** — igual
  à diferença entre a nossa FK e a do robô (sobretudo z ≈ −1.1 mm): é a
  calibração de fábrica, não imprecisão da IK;
- nalguns pontos a IK do robô escolhe o **pulso virado** (J4 ≈ ±180°, J5 < 0):
  mesma pose, outros ângulos.

---

## 10. Comandos

```bash
python src/kinematics.py --validar                      # §1.4
python src/trajectory.py --plot                         # §3–§6, gráficos em report/
python src/run_robot.py                                 # ensaio da Tarefa 3
python src/run_robot.py --go --lento 3                  # pontos, devagar
python src/run_robot.py --go --metodo servo --lento 2   # servo, grava dados
python src/ik_compare.py --n 60                         # Tarefa 4 (não mexe o robô)
```

**Falta:** repetir no lab `servo --lento 2` com φ_end 50° (o ficheiro completo
foi escrito por cima) e escrever o relatório.
