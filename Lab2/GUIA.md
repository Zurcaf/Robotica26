# Guia do Lab 2 — tudo o que foi feito e porquê

Este guia explica, do zero, o workspace do Lab 2: o que o enunciado pede, como
está organizado o código, a teoria por trás de cada ficheiro, as decisões
tomadas (e porquê), o que já foi testado e o que ainda falta fazer no
laboratório.

> Resumo numa frase: a garra do Lite 6 faz de cabeça do taco do Lab 1 — percorre
> um arco a passar na bola, com o "pulso" a soltar tarde — e há código para
> (1) a cinemática direta, (2) gerar e verificar essa trajetória, (3) mandá-la
> para o robô e (4) medir a precisão da cinemática inversa do robô.

---

## Índice

1. [O enunciado em resumo](#1-o-enunciado-em-resumo)
2. [Onde está cada coisa](#2-onde-está-cada-coisa)
3. [Setup do ambiente](#3-setup-do-ambiente)
4. [Convenções usadas em todo o código](#4-convenções-usadas-em-todo-o-código)
5. [config.py — números do fabricante](#5-configpy--números-do-fabricante)
6. [Tarefa 1 — kinematics.py (cinemática direta)](#6-tarefa-1--kinematicspy-cinemática-direta)
7. [Tarefa 2 — trajectory.py (o swing)](#7-tarefa-2--trajectorypy-o-swing)
8. [A singularidade que apareceu (e como se resolveu)](#8-a-singularidade-que-apareceu-e-como-se-resolveu)
9. [robot.py — ligação ao robô](#9-robotpy--ligação-ao-robô)
10. [Tarefa 3 — run_robot.py (executar no robô)](#10-tarefa-3--run_robotpy-executar-no-robô)
11. [Tarefa 4 — ik_compare.py (precisão da IK)](#11-tarefa-4--ik_comparepy-precisão-da-ik)
12. [O que foi testado e o que não foi](#12-o-que-foi-testado-e-o-que-não-foi)
13. [Checklist para o dia do laboratório](#13-checklist-para-o-dia-do-laboratório)
14. [Como mexer nos parâmetros](#14-como-mexer-nos-parâmetros)
15. [Do código ao relatório](#15-do-código-ao-relatório)
16. [Limitações conhecidas e ideias de melhoria](#16-limitações-conhecidas-e-ideias-de-melhoria)
17. [Glossário](#17-glossário)

---

## 1. O enunciado em resumo

Robô: **UFactory Lite 6**, braço série com **6 juntas rotacionais**, ligado a um
portátil por cabo Ethernet (**só um portátil de cada vez**), programado em
Python com o SDK oficial (`xarm-python-sdk`). Entrega: **13/11/2026, 23:59:59**,
grupos até 3.

| Tarefa | O que pede | Ficheiro |
|---|---|---|
| 1 | Cinemática direta do Lite 6. Explicar a atribuição de referenciais e o método. | `src/kinematics.py` |
| 2 | Trajetória (posição **e** orientação da garra em função do tempo) análoga ao swing do Lab 1, adaptada ao tamanho do robô. Bola fixa num ponto alcançável. Tem de ser **viável**: (i) não exceder velocidades do fabricante, (ii) não sair da zona alcançável, (iii) não violar limites das juntas. Gráficos claros que mostrem que não há contacto perigoso. | `src/trajectory.py` |
| 3 | Script Python que põe o robô a seguir a trajetória, com as funções do SDK. | `src/run_robot.py` |
| 4 | Para muitos pontos da trajetória, obter as juntas com a **IK do robô**; aplicar-lhes a **FK da Tarefa 1**; comparar com a trajetória original e concluir sobre a precisão da IK do robô. | `src/ik_compare.py` |

---

## 2. Onde está cada coisa

```
Robotica26/                 (repositório git do grupo, remoto Zurcaf/Robotica26)
├── Lab1/                   (o lab anterior — não foi tocado)
└── Lab2/
    ├── GUIA.md             ← este ficheiro
    ├── lab2.ipynb          notebook com tudo: teoria + código a correr + gráficos
    ├── README.md           resumo curto + comandos
    ├── lab_2_2026.pdf      enunciado (copiado de Downloads)
    ├── requirements.txt    numpy, matplotlib, xarm-python-sdk
    ├── .venv/              ambiente Python do Lab 2 (ignorado pelo git)
    ├── src/
    │   ├── config.py       IP do robô, limites do fabricante, mesa/base
    │   ├── kinematics.py   Tarefa 1 (+ IK [INVERSE KINEMATICS] numérica para verificação)
    │   ├── trajectory.py   Tarefa 2
    │   ├── robot.py        ligação ao robô (comum às Tarefas 3 e 4)
    │   ├── run_robot.py    Tarefa 3
    │   └── ik_compare.py   Tarefa 4
    ├── data/               CSVs recolhidos no laboratório (vazia por agora)
    ├── report/
    │   ├── relatorio.tex   esqueleto do relatório, uma secção por tarefa
    │   ├── traj_cartesiana.png   posição, orientação e |v| da garra vs t
    │   ├── traj_juntas.png       ângulos e velocidades das 6 juntas vs t
    │   └── traj_3d.png           caminho 3D da garra, bola e base
    └── scripts_curso/      para pôr os scripts de exemplo da página da cadeira
```

**Como as peças encaixam:**

```
config.py ──► kinematics.py ──► trajectory.py ──► run_robot.py  (Tarefa 3)
                    │                 │
                    └─────────────────┴──────────► ik_compare.py (Tarefa 4)
                                                        ▲
                                     robot.py ──────────┘ (e run_robot.py)
```

- `kinematics.py` não depende do robô: é matemática pura.
- `trajectory.py` usa a FK e uma IK numérica própria para conseguir verificar
  limites de juntas **sem** robô.
- Só `run_robot.py` e `ik_compare.py` falam com o robô, sempre via `robot.py`.

**Estado no git:** nada foi commitado. O `Lab1` tem alterações por commitar que
já lá estavam antes — não foram mexidas.

---

## 3. Setup do ambiente

Já está feito neste computador: existe `Lab2/.venv` com tudo instalado. Noutro
computador (por exemplo o portátil que vai ao laboratório):

```bash
cd Lab2
python -m venv .venv
.venv\Scripts\activate         # Windows   (Mac/Linux: source .venv/bin/activate)
pip install -r requirements.txt
```

Precisa de Python ≥ 3.10. Todos os comandos correm **a partir da pasta `Lab2/`**.

Sem ativar a venv, no Windows também dá para correr diretamente:

```bash
.venv\Scripts\python.exe src\trajectory.py --plot
```

---

## 4. Convenções usadas em todo o código

São as **mesmas do SDK do xArm**, de propósito, para que nada precise de
conversão entre o nosso código e o robô:

| Grandeza | Unidade / convenção |
|---|---|
| Posição | milímetros (mm), no referencial da **base** do robô |
| Ângulos de junta | graus |
| Orientação | roll, pitch, yaw em graus, com **R = Rz(yaw) · Ry(pitch) · Rx(roll)** |
| Pose (formato SDK) | `[x, y, z, roll, pitch, yaw]` |
| Referencial da base | z para cima, origem no fundo da base, x para a frente do robô |

O SDK é criado com `is_radian=False`, por isso tudo o que entra e sai do robô
está em graus.

Detalhe: o SDK quer ângulos em `(-180, 180]`. A trajetória, internamente, usa
ângulos "desembrulhados" (o roll pode ir até 255°, ver gráfico) para os
gráficos serem contínuos; a função `wrap180()` de `robot.py` volta a pô-los no
intervalo certo antes de os enviar.

---

## 5. `config.py` — números do fabricante

Juntei num só sítio todos os números que vêm da datasheet, para o relatório
poder citar a fonte e para só haver um sítio a corrigir.

| Constante | Valor | Significado |
|---|---|---|
| `ROBOT_IP` | `192.168.1.185` (ou variável de ambiente `LITE6_IP`) | IP da caixa de controlo — **placeholder, confirmar na etiqueta** |
| `JOINT_LIMITS` | J1 ±360°, J2 ±150°, J3 −3.5…300°, J4 ±360°, J5 ±124°, J6 ±360° | limites físicos das juntas |
| `JOINT_SPEED_MAX` | 180 °/s | velocidade máxima de cada junta |
| `TCP_SPEED_MAX` | 500 mm/s | velocidade linear máxima da ferramenta |
| `REACH` | 440 mm | alcance máximo |
| `SAFETY_FACTOR` | 0.8 | só se aceitam trajetórias que usem ≤ 80 % dos limites |
| `TABLE_Z`, `TABLE_CLEARANCE` | 0 mm, 30 mm | a garra tem de ficar ≥ 30 mm acima da mesa |
| `BASE_RADIUS` | 80 mm | cilindro à volta do eixo de J1 onde a garra não pode entrar |

**Atenção:** os valores do fabricante foram escritos de memória e estão
marcados `CONFIRMAR`. Antes de entregar, confirmar na datasheet/manual
oficial e corrigir aqui — tudo o resto se ajusta sozinho.

Porquê a margem de 80 %: os limites do fabricante são máximos absolutos; uma
trajetória que os toque exatamente pode disparar erros de proteção do
controlador. A margem também cobre o facto de as velocidades serem calculadas
numericamente (diferenças finitas).

---

## 6. Tarefa 1 — `kinematics.py` (cinemática direta)

### 6.1 O problema

Cinemática direta (FK, *forward kinematics*): dados os 6 ângulos das juntas
`q = (q1…q6)`, calcular a **posição e orientação** do flange (ponta do robô)
no referencial da base.

### 6.2 O método: Denavit-Hartenberg **modificado** (Craig)

Coloca-se um referencial em cada junta e descreve-se a passagem de cada um
para o seguinte com só **4 parâmetros**. Na convenção modificada, o
referencial `{i}` fica **na junta i** (o eixo z_i é o eixo de rotação da junta
i) e:

```
{i-1}T{i} = Rx(α_{i-1}) · Tx(a_{i-1}) · Rz(θ_i) · Tz(d_i)
```

ou seja, por esta ordem:
1. rodar `α_{i-1}` em torno de x_{i-1} (alinha o eixo z com a nova junta),
2. andar `a_{i-1}` ao longo de x_{i-1} (distância entre eixos),
3. rodar `θ_i` em torno de z_i (**é aqui que entra a variável de junta**),
4. andar `d_i` ao longo de z_i (desvio ao longo do eixo).

Com `θ_i = q_i + offset_i`. O offset existe porque o "zero" mecânico do robô
não coincide com o zero natural da convenção DH.

A matriz de cada elo (implementada em `link_transform()`):

```
        ⎡ cosθ        −sinθ        0        a     ⎤
T  =    ⎢ sinθ·cosα    cosθ·cosα  −sinα   −sinα·d ⎥
        ⎢ sinθ·sinα    cosθ·sinα   cosα    cosα·d ⎥
        ⎣ 0            0           0        1     ⎦
```

E a FK completa é o produto:

```
{0}T{6} = {0}T{1} · {1}T{2} · {2}T{3} · {3}T{4} · {4}T{5} · {5}T{6}
```

A parte 3×3 de cima à esquerda é a **rotação** do flange; a última coluna é a
**posição** (mm).

**Porquê a convenção modificada e não a clássica?** Porque é a que a UFactory
usa na documentação dos parâmetros cinemáticos do Lite 6 — assim os nossos
resultados batem diretamente com os do controlador, que é precisamente o que
a Tarefa 4 vai comparar.

### 6.3 A tabela DH usada

| Junta | offset θ | d [mm] | α_{i-1} | a_{i-1} [mm] |
|---|---|---|---|---|
| J1 | 0° | 243.3 | 0° | 0 |
| J2 | −90° | 0 | −90° | 0 |
| J3 | −90° | 0 | 180° | 200 |
| J4 | 0° | 227.6 | 90° | 87 |
| J5 | 0° | 0 | 90° | 0 |
| J6 | 0° | 61.5 | −90° | 0 |

Leitura física:
- **243.3 mm** — altura do ombro (J2) acima da base.
- **200 mm** — comprimento do "braço" (J2 → J3).
- **87 mm** e **227.6 mm** — o antebraço tem um desvio lateral de 87 mm e
  227.6 mm de comprimento até ao pulso.
- **61.5 mm** — do centro do pulso ao flange.
- J4, J5, J6 formam um **pulso esférico** (os três eixos cruzam-se num ponto).

### 6.4 Validação feita

Com todas as juntas a zero:

```bash
python src/kinematics.py
# pose flange  : x=87.00 y=-0.00 z=154.20 mm  roll=180.00 pitch=0.00 yaw=-0.00 deg
```

Bate com a pose de zero conhecida do Lite 6: x = 87 (o desvio do antebraço),
z = 243.3 + 200 − 227.6 − 61.5 = 154.2 mm (o braço sobe 200 e o antebraço e
o pulso descem), garra virada para baixo (roll 180°). É um bom indício de
que a tabela está certa — a validação definitiva é na Tarefa 4, comparando
com `arm.get_forward_kinematics()` do controlador.

Outros comandos:

```bash
python src/kinematics.py --q 0 30 60 0 30 0   # FK para outros ângulos
python src/kinematics.py --frames             # imprime as 7 matrizes {0}T{i}
```

O `--frames` serve para desenhar a figura dos referenciais no relatório
(o enunciado pede explicitamente "explain clearly the frame assignment").

### 6.5 As outras funções do ficheiro

| Função | O que faz |
|---|---|
| `fk(q, all_frames=False)` | devolve `{0}T{6}` (4×4), ou a lista de todos os `{0}T{i}` |
| `fk_pose(q)` | devolve `[x, y, z, roll, pitch, yaw]` como o SDK |
| `rpy_from_R(R)` / `R_from_rpy(r,p,y)` | conversão entre matriz de rotação e roll/pitch/yaw (convenção do SDK) |
| `rotvec(R)` | "logaritmo" de uma rotação: o vetor eixo×ângulo equivalente. Usado para medir erros de orientação |
| `orientation_error_deg(Ra, Rb)` | ângulo (graus) entre duas orientações — um só número, independente da convenção rpy |
| `ik_numeric(T, q0)` | **IK numérica própria** (ver abaixo) |

### 6.6 A IK numérica (porque existe, se o enunciado pede a do robô?)

A Tarefa 2 exige verificar que a trajetória não viola os limites das juntas
nem as velocidades das juntas. Para isso é preciso saber os ângulos das
juntas em cada ponto — ou seja, uma IK. A IK do robô (a da Tarefa 4) só corre
**com o robô ligado**. Para se poder desenhar e verificar a trajetória em
casa, fiz uma IK numérica simples. **Não substitui** a do robô na Tarefa 4.

Método: **mínimos quadrados amortecidos** (*damped least squares*,
Levenberg-Marquardt):

1. erro `e` = (erro de posição em mm, erro de orientação em rad × 100);
2. jacobiano `J` (6×6) por diferenças finitas: mexe-se cada junta 0.0001° e
   vê-se quanto muda o erro;
3. passo `Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e` — o termo `λ²I` impede passos enormes
   perto de singularidades;
4. corta-se `q` aos limites das juntas e repete-se até o erro ser < 0.001 mm
   e < 0.00001 rad (até 200 iterações).

O peso 100 na orientação põe 1 rad de erro de orientação ao nível de 100 mm
de erro de posição, para as duas partes contarem de forma equilibrada.

Ao percorrer a trajetória, cada ponto usa como ponto de partida a solução do
ponto anterior — isso garante que a IK escolhe sempre o mesmo "ramo" de
solução (cotovelo/pulso na mesma configuração) e que as juntas variam de
forma contínua.

---

## 7. Tarefa 2 — `trajectory.py` (o swing)

### 7.1 A ideia: transpor o pêndulo duplo do Lab 1

No Lab 1 o swing era um **pêndulo duplo**: o braço roda em torno do ombro e o
taco roda em torno do pulso; o pulso fica "armado" e só solta tarde (o *lag*),
o que dá velocidade. Aqui a **garra faz de cabeça do taco**:

| Lab 1 | Lab 2 |
|---|---|
| braço a rodar em torno do ombro | a garra percorre um **arco de raio R** num plano vertical, centrado num "ombro virtual" por cima da bola. Ângulo `phi`, com `phi = 0` na bola |
| pulso armado / a soltar | a orientação da garra roda `phi + psi` em torno da normal ao plano; `psi` é o "pulso": armado no topo, solto a `f_release` do downswing |
| bola no chão em (0,0) | bola fixa em `bola = (250, 0, 80)` mm |

Posição da garra ao longo do tempo:

```
centro = bola + R·ẑ                               (ombro virtual R mm acima da bola)
p(t)   = centro + R·( sin(phi)·d̂  −  cos(phi)·ẑ )
```

- `d̂` = direção do alvo (por omissão +y, horizontal);
- com `phi = 0` → `p = bola` (impacto); com `phi = −100°` → atrás e acima (topo).

Orientação:

```
R(t) = Rot(−n̂, phi + psi) · R_endereço
```

- `n̂ = d̂ × ẑ` é a normal ao plano do swing;
- `R_endereço` é a orientação da garra na bola (`rpy_enderecio`, ver secção 8).

### 7.2 As três fases (iguais às do Lab 1)

**1. Backswing** (0 → `t_back`): `phi` vai de 0 a `phi_top` com um perfil
**quíntico**

```
s(τ) = 10τ³ − 15τ⁴ + 6τ⁵      (τ = t / t_back)
```

que parte e chega com velocidade **e** aceleração nulas (movimento suave, sem
solavancos). O pulso arma ao mesmo tempo (`psi`: 0 → `psi_top`).

**2. Downswing** (`t_back` → `t_back + t_down`): **aceleração constante**,
exatamente como no Lab 1:

```
phi(u) = phi_top · (1 − (u/T)²)        u = tempo desde o topo, T = t_down
```

- em `u = 0`: `phi = phi_top`, velocidade 0 (parte parado do topo — encaixa
  com o fim do backswing);
- em `u = T`: `phi = 0` (**bola**) e a velocidade é **máxima**:
  `ω_impacto = −2·phi_top / T`.

O pulso fica armado até `f_release·T` e depois solta (quíntico) até `psi = 0`
exatamente no impacto — a garra chega à bola com a orientação de endereço.

**3. Follow-through**: desaceleração constante até parar em `phi_end`. A
duração sai das contas: `T_f = 2·phi_end / ω_impacto`.

### 7.3 Valores por omissão (`DEFAULTS`)

| Parâmetro | Valor | Significado |
|---|---|---|
| `bola` | (250, 0, 80) mm | posição da bola |
| `direcao` | (0, 1, 0) | direção do alvo (+y) |
| `raio` | 150 mm | raio do arco ("braço + taco") |
| `phi_top` | −100° | topo do backswing |
| `phi_end` | 80° | fim do follow-through |
| `psi_top` | 30° | quanto o "pulso" arma |
| `f_release` | 0.6 | o pulso solta aos 60 % do downswing |
| `t_back` | 2.0 s | duração do backswing |
| `t_down` | 2.2 s | duração do downswing |
| `rpy_enderecio` | (180, 45, 0)° | orientação da garra na bola (secção 8) |
| `dt` | 0.01 s | amostragem (100 Hz) |

Contas com estes valores:
- `ω_impacto = 2 × 1.745 rad / 2.2 s = 1.59 rad/s` → velocidade da garra na
  bola `= 1.59 × 150 = 238 mm/s`;
- `T_f = 2 × 1.396 / 1.59 = 1.76 s`;
- duração total `= 2.0 + 2.2 + 1.76 = 5.96 s` (597 pontos a 100 Hz).

### 7.4 A verificação de viabilidade (`check_feasibility`)

O enunciado pede três coisas e mais "sem contacto perigoso". O código verifica
seis:

| Verificação | Como | Exigência do enunciado |
|---|---|---|
| velocidade da garra | `|dp/dt|` por diferenças finitas, máximo ≤ 0.8 × 500 mm/s | (i) |
| velocidade das juntas | `|dq/dt|` (q da IK numérica), máximo ≤ 0.8 × 180 °/s | (i) |
| alcance | distância ao ombro (origem de `{2}`) ≤ 0.8 × 440 mm | (ii) |
| limites das juntas | todos os q dentro de `JOINT_LIMITS` | (iii) |
| acima da mesa | z da garra ≥ 30 mm | contacto perigoso |
| fora da base | distância ao eixo de J1 ≥ 80 mm | contacto perigoso |

E ainda: se a IK numérica **não convergir** num ponto, esse ponto é
considerado inalcançável e a trajetória é marcada como não viável.

Resultado atual:

```bash
python src/trajectory.py --plot
#   597 pontos, 5.96 s, IK numérica convergiu
#   vel. TCP        : max    245.4 mm/s   (limite c/ margem 400.0)  OK
#   vel. juntas     : max    123.0 deg/s  (limite c/ margem 144.0)  OK
#   alcance         : max    298.6 mm     (limite c/ margem 352.0)  OK
#   limites juntas  : OK
#   acima da mesa   : z min 80.0 mm  OK
#   fora da base    : r min 250.0 mm  OK
#   => trajetória VIÁVEL
```

### 7.5 Os gráficos (em `report/`)

- **`traj_cartesiana.png`** — 3 painéis: x, y, z; roll, pitch, yaw; e |v| da
  garra com a linha vermelha do limite do fabricante. Lê-se bem o swing: y vai
  a −150 (backswing), z sobe a ~255 (topo), volta a z = 80 (bola) por volta
  de t = 4.2 s, onde a velocidade tem o pico do downswing (forma triangular =
  aceleração constante, depois desaceleração constante).
- **`traj_juntas.png`** — os 6 ângulos e as 6 velocidades das juntas, com as
  linhas de ±180 °/s.
- **`traj_3d.png`** — o caminho da garra em 3D, a bola e dois círculos a marcar
  o cilindro da base: mostra que o caminho passa longe do robô.

Estes três gráficos respondem ao "provide clear plots… no dangerous physical
contact".

---

## 8. A singularidade que apareceu (e como se resolveu)

Isto é uma das coisas mais interessantes para o relatório.

**Primeira tentativa:** garra a apontar a pique para a bola
(`rpy = (180, 0, 0)`), `t_down = 1.5 s`. Resultado:

```
IK numérica NÃO convergiu em 61 pontos
vel. juntas : max 1908 deg/s   FALHA
```

**Diagnóstico:** com a garra vertical, o eixo de J6 fica alinhado com o eixo
de J4 — isto acontece quando **J5 = 0**. É a **singularidade de pulso** dos
pulsos esféricos: J4 e J6 passam a fazer a mesma coisa, há infinitas
combinações que dão a mesma orientação, e qualquer pequena mudança de
orientação perto daí exige que J4 e J6 rodem muito depressa em sentidos
opostos (daí os ~1900 °/s em J4 e J6). Matematicamente, o jacobiano perde
característica (deixa de ser invertível).

**Solução:** inclinar a garra 45° na pose de endereço,
`rpy_enderecio = (180, 45, 0)`. J5 passa a variar entre ~53° e ~115° ao longo
de todo o swing — sempre longe de 0. Testei também ±30°:

| rpy de endereço | máx. vel. de cada junta [°/s] (J1…J6) |
|---|---|
| (180, 30, 0) | 96, 26, 29, 113, 102, **241** |
| (180, −30, 0) | 123, 33, 31, **301**, 82, 204 |
| (180, 45, 0) | 88, 26, 24, 40, 83, **180** |

(estes valores ainda com `t_down = 1.5 s`)

Com 45° ainda faltava: J6 a 180 °/s, acima dos 144 °/s permitidos com margem.
J6 é a junta que segue a rotação da garra, que é proporcional à velocidade do
swing. Por isso **aumentou-se `t_down` de 1.5 para 2.2 s**: a velocidade
escala com 1/T, ou seja 180 × 1.5/2.2 ≈ 123 °/s. Fica dentro.

**Conclusão para o relatório:** neste robô o limite de velocidade do swing não
vem da velocidade linear da garra (238 mm/s ≪ 500 mm/s), mas da velocidade da
**junta 6**, e a orientação da garra tem de ser escolhida para evitar a
singularidade do pulso.

---

## 9. `robot.py` — ligação ao robô

Pequeno módulo partilhado pelas Tarefas 3 e 4:

| Função | O que faz |
|---|---|
| `connect(ip, enable_motion=True)` | cria o `XArmAPI`, limpa avisos/erros; se `enable_motion`, ativa os motores, põe **modo 0** (posição) e **estado 0** (pronto). Com `enable_motion=False` só liga — serve para pedir IK/FK sem mexer |
| `wrap180(a)` | põe ângulos em (−180, 180] |
| `pose_sdk(pos, rpy)` | monta `[x, y, z, roll, pitch, yaw]` para o SDK |
| `check(code, what)` | o SDK devolve código 0 se correu bem; qualquer outro valor levanta erro com o nome da função — evita que um erro passe em silêncio |

Modos do xArm referidos no código:
- **modo 0** — posição: mandam-se alvos e o controlador planeia o movimento;
- **modo 1** — servo: mandam-se pontos a alta frequência e o robô vai direto a
  cada um (é preciso ser o programa a fazer o perfil temporal).

---

## 10. Tarefa 3 — `run_robot.py` (executar no robô)

### 10.1 Fluxo

1. Gera a trajetória da Tarefa 2, **esticada** pelo fator `--lento`
   (por omissão 3: `t_back` e `t_down` × 3 → tudo 3× mais devagar).
2. Corre `check_feasibility`. **Se falhar, sai sem tocar no robô.**
3. Mostra a pose inicial e a duração.
4. Sem `--go`: é só um **ensaio** e acaba aqui.
5. Com `--go`: pede para escrever `sim` (robô livre, emergência à mão).
6. Liga, vai até à pose inicial a 50 mm/s (`set_position(..., wait=True)`).
7. Executa a trajetória por um de dois métodos.
8. Desliga sempre (`finally: arm.disconnect()`), mesmo que haja erro.

### 10.2 Os dois métodos

**`--metodo pontos`** (por omissão, mais robusto)
- Fica no modo 0. Pega num ponto a cada 10 (de 0.1 em 0.1 s na trajetória
  original) e manda cada um com `set_position(..., radius=5, wait=False)`.
- `radius=5` faz *blending*: o controlador arredonda a passagem por cada
  ponto (raio 5 mm) em vez de parar em cada um.
- A velocidade de cada segmento é a velocidade média da trajetória nesse
  troço → o perfil do swing (lento no topo, rápido na bola) é aproximado.
- Desvantagem: quem gera o movimento entre pontos é o controlador, por isso
  o perfil temporal não é exatamente o planeado.

**`--metodo servo`** (segue o perfil à risca)
- Passa a modo 1 e manda **um ponto a cada `dt`** (10 ms × fator lento) com
  `set_servo_cartesian()`; espera ativamente até ao instante certo.
- Em cada passo lê a pose real (`get_position()`) e no fim grava
  `data/run_servo.csv` com referência e pose medida lado a lado → dá para fazer
  um gráfico "referência vs real" no relatório.
- No fim volta a modo 0.

### 10.3 Comandos

```bash
python src/run_robot.py                         # ensaio, não mexe
python src/run_robot.py --go --lento 3          # primeira vez no robô
python src/run_robot.py --go --lento 1          # velocidade planeada
python src/run_robot.py --go --metodo servo --lento 2
python src/run_robot.py --go --ip 192.168.1.xxx # outro IP
```

Ensaio já testado (offline):

```
  vel. TCP : max 81.8 mm/s ... OK      (3× mais lento)
  início: [ 250. 0. 80. -180. 45. 0.]   duração: 17.9 s   método: pontos
  ENSAIO — nada foi enviado. Usa --go para executar.
```

### 10.4 Segurança — porque está assim

- Ensaio por omissão: correr o script "sem querer" não mexe o robô.
- Recusa trajetórias que falhem a verificação.
- Confirmação explícita antes de mexer.
- Aproximação lenta à pose inicial (50 mm/s), porque o robô pode estar em
  qualquer posição antes.
- Começar sempre com `--lento 3` e baixar aos poucos.
- **Mão no botão de emergência** sempre.

---

## 11. Tarefa 4 — `ik_compare.py` (precisão da IK)

### 11.1 O procedimento

Para `N` pontos espalhados uniformemente por **toda** a trajetória (por omissão
60):

1. `q_sdk = arm.get_inverse_kinematics(pose_ref)` — a **IK do robô** (corre no
   controlador);
2. `pose_fk = fk_pose(q_sdk)` — a **nossa FK** da Tarefa 1;
3. erro de posição `|p_fk − p_ref|` em mm, e erro de orientação (ângulo da
   rotação entre as duas orientações, em graus).

Extra: também se pede `arm.get_forward_kinematics(q_sdk)` — a FK do próprio
controlador. Se a nossa FK coincidir com a dele, **a tabela DH da Tarefa 1
fica validada experimentalmente**. Útil para separar as duas fontes de erro:

| Se… | então… |
|---|---|
| nossa FK = FK do robô, e erro pequeno | a IK do robô é precisa e a nossa FK está certa |
| nossa FK = FK do robô, mas erro grande nalguns pontos | a IK do robô tem imprecisões ali (ver se é perto de singularidades/limites) |
| nossa FK ≠ FK do robô | há um erro na tabela DH (ou a ferramenta tem offset) — corrigir antes de concluir sobre a IK |

O script **não ativa os motores** (`enable_motion=False`) — só consulta o
controlador, por isso é seguro.

### 11.2 Saídas

- `data/ik_compare.csv` — por ponto: t, pose de referência, q do SDK, pose
  da nossa FK, pose da FK do SDK;
- `report/ik_erros.png` — erro de posição e de orientação ao longo do tempo;
- no terminal: médias e máximos.

```bash
python src/ik_compare.py --n 60        # no laboratório
python src/ik_compare.py --from-csv    # em casa, a partir do CSV
```

### 11.3 Teste feito

Sem robô não dá para chamar a IK do controlador. Testei a parte da análise
substituindo a IK do robô pela IK numérica: deu erros ~0.0004 mm / 0.0004°, o
que confirma que a análise e o gráfico funcionam. **Esses números não são o
resultado da Tarefa 4** — o resultado real só sai no laboratório.

### 11.4 O que discutir no relatório

- ordem de grandeza dos erros (mm? µm?) face à repetibilidade do robô
  (datasheet, ~±0.2 mm);
- onde estão os maiores erros (topo do backswing? perto da bola?) e se
  coincidem com J5 pequeno (singularidade) ou juntas perto dos limites;
- se a IK do robô devolve a mesma configuração (cotovelo/pulso) ao longo da
  trajetória — a pose pode estar certa com juntas diferentes das da nossa IK;
- se houve pontos em que a IK do robô falhou (código ≠ 0).

---

## 12. O que foi testado e o que não foi

| Coisa | Estado |
|---|---|
| Ambiente `.venv` com numpy, matplotlib, xarm-python-sdk | ✅ instalado |
| FK com q = 0 → (87, 0, 154.2) mm, roll 180° | ✅ bate com o valor conhecido |
| Geração da trajetória + verificação | ✅ VIÁVEL com os defaults |
| Gráficos da Tarefa 2 | ✅ gerados em `report/` |
| `run_robot.py` em modo ensaio | ✅ |
| `run_robot.py --go` (pontos / servo) | ❌ **não testado — precisa do robô** |
| `ik_compare.py` análise e gráfico | ✅ com dados simulados |
| `ik_compare.py` a falar com o robô | ❌ **não testado — precisa do robô** |
| Tabela DH e limites confirmados na documentação oficial | ❌ **por confirmar** |
| Commit no git | ❌ nada commitado |

---

## 13. Checklist para o dia do laboratório

Antes de ir:
- [ ] Confirmar tabela DH e limites na documentação oficial do Lite 6; corrigir
      `config.py` / `kinematics.py` se for preciso e voltar a correr
      `python src/trajectory.py --plot`.
- [ ] Levar o portátil com a `.venv` pronta (ou instalar antes de chegar).
- [ ] Copiar para `scripts_curso/` os exemplos da página da cadeira.

No laboratório:
1. [ ] Ler o IP na caixa de controlo; pôr o portátil com IP fixo na mesma
       sub-rede (ex.: 192.168.1.100, máscara 255.255.255.0).
2. [ ] `set LITE6_IP=<ip>` (PowerShell: `$env:LITE6_IP="<ip>"`), ou `--ip`.
3. [ ] Correr um exemplo do curso para confirmar a ligação.
4. [ ] Ver se há garra/ferramenta montada e se o controlador tem um offset de
       ferramenta (TCP offset) configurado. Se sim, a pose que o robô reporta
       não é a do flange — ter isso em conta na Tarefa 4.
5. [ ] Medir/decidir a altura da mesa e a posição da bola; ajustar `bola` e
       `TABLE_Z` e verificar outra vez.
6. [ ] **Tarefa 4 primeiro** (não mexe o robô): `python src/ik_compare.py --n 60`.
       Ver logo se a nossa FK coincide com a do controlador.
7. [ ] Tarefa 3: `python src/run_robot.py` (ensaio) → `--go --lento 3` →
       baixar o fator aos poucos → `--metodo servo` para gravar o CSV.
8. [ ] Filmar / fotografar o robô a fazer o swing para o relatório.
9. [ ] Guardar os CSVs de `data/` antes de sair (e commitar).

---

## 14. Como mexer nos parâmetros

Os parâmetros estão em `DEFAULTS`, no topo de `src/trajectory.py`. Pela linha
de comandos só há dois atalhos (`--t-down`, `--raio`); para os outros, edita
`DEFAULTS`. Depois de qualquer alteração: `python src/trajectory.py --plot`.

| Quero… | Mexer em | Efeito / cuidado |
|---|---|---|
| swing mais rápido | `t_down` ↓ | velocidades ∝ 1/T; J6 é o primeiro a passar o limite |
| backswing mais lento (mais realista) | `t_back` ↑ | atualmente o pico do backswing (245 mm/s) é maior que o do impacto (238 mm/s) — num swing real o backswing é mais lento |
| arco maior | `raio` ↑ | mais velocidade linear para o mesmo ω, mas cuidado com o alcance e com o topo |
| backswing mais alto | `phi_top` mais negativo | aumenta ω de impacto e o alcance no topo |
| "pulso" mais ativo | `psi_top` ↑, `f_release` ↑ | mais rotação da garra → mais carga em J6 |
| bola noutro sítio | `bola` | confirmar alcance, mesa e base |
| swing noutra direção | `direcao` | ex.: (1, 0, 0) faz o swing em direção oposta à base — rever a base |
| outra orientação de endereço | `rpy_enderecio` | manter J5 longe de 0 (ver secção 8) |
| mais/menos exigente | `SAFETY_FACTOR` em `config.py` | 0.8 = 80 % dos limites |

---

## 15. Do código ao relatório

O esqueleto `report/relatorio.tex` já tem uma secção por tarefa com
comentários do que lá pôr. Correspondência:

| Secção | Material |
|---|---|
| Tarefa 1 | figura dos referenciais (desenhar à mão ou a partir do `--frames`), tabela DH (secção 6.3), fórmula da secção 6.2, validação q = 0 e comparação com a FK do controlador |
| Tarefa 2 | analogia com o Lab 1 (7.1), equações das fases (7.2), tabela de parâmetros (7.3), tabela de verificações (7.4), os 3 gráficos, **a história da singularidade (secção 8)** |
| Tarefa 3 | método escolhido e porquê, fator de abrandamento, gráfico referência vs real a partir de `data/run_servo.csv`, fotos/vídeo |
| Tarefa 4 | procedimento (11.1), `ik_erros.png`, tabela de médias/máximos, discussão (11.4) |

O autor está como "Grupo 1" (por causa do `GR1_Lab1.zip`) — trocar pelos
nomes.

---

## 16. Limitações conhecidas e ideias de melhoria

- **Valores do fabricante não confirmados** (tabela DH, limites, velocidades,
  IP). É a primeira coisa a fazer.
- **A bola é um ponto.** A garra passa exatamente na posição da bola;
  não há modelo do tamanho da bola nem da garra. Se houver uma bola física,
  pôr a posição da bola uns mm ao lado/acima, ou definir um offset de
  ferramenta.
- **Colisões simplificadas:** só se verifica a ponta (flange) contra a mesa e
  um cilindro à volta da base. Os elos do robô (cotovelo, antebraço) não são
  verificados. Uma melhoria fácil: usar `fk(q, all_frames=True)` para
  verificar também a altura de cada junta acima da mesa.
- **Aceleração descontínua** na passagem downswing → follow-through (é o que
  dá o "bico" no gráfico de velocidade). É fiel ao Lab 1, mas pode fazer o
  robô dar um pequeno solavanco; se for um problema, suavizar essa transição.
- **Acelerações das juntas não são verificadas** — só velocidades. O
  enunciado só pede velocidades, mas o controlador também tem limites de
  aceleração.
- **Modo servo:** a chamada `get_position()` em cada passo pode atrasar o ciclo
  de 10 ms. Se o robô andar "aos soluços", reduzir as leituras (ler 1 em
  cada 5) ou usar `--lento` maior.
- **Modo pontos:** a velocidade de cada segmento é uma média, por isso o pico
  na bola fica um pouco suavizado.
- **IK do robô sem ponto de referência:** `get_inverse_kinematics` é chamada
  sem indicar a configuração anterior; o controlador pode devolver uma
  configuração diferente da nossa. Para a Tarefa 4 não interessa (compara-se
  a pose, não as juntas), mas vale a pena comentar.
- A IK numérica demora poucos segundos para os 597 pontos, mas ~30 s quando
  falha perto de singularidades (esgota as 200 iterações). Se incomodar,
  implementar o jacobiano analítico.

---

## 17. Glossário

| Termo | Significado |
|---|---|
| **FK** (cinemática direta) | juntas → pose da ferramenta |
| **IK** (cinemática inversa) | pose da ferramenta → juntas (pode ter várias soluções ou nenhuma) |
| **DH** | Denavit-Hartenberg: 4 parâmetros por elo para descrever um robô série |
| **Flange** | a "ponta" do último elo, onde se monta a garra |
| **TCP** | *Tool Center Point*: o ponto de trabalho da ferramenta (flange + offset da ferramenta) |
| **Pose** | posição + orientação |
| **roll / pitch / yaw** | rotações em torno de x / y / z (aqui R = Rz·Ry·Rx) |
| **Jacobiano** | matriz que relaciona velocidades das juntas com velocidade da ferramenta |
| **Singularidade** | configuração em que o jacobiano perde característica: há direções em que a ferramenta não se consegue mexer, e perto disso as juntas precisam de velocidades enormes |
| **Pulso esférico** | três últimas juntas com eixos a cruzar num ponto (J4, J5, J6) |
| **Quíntico** | polinómio de grau 5 que liga 0 a 1 com velocidade e aceleração nulas nos extremos |
| **Blending** | passar perto de um waypoint sem parar, arredondando a esquina |
| **Modo 0 / modo 1** | xArm: posição (o controlador planeia) / servo (o programa manda pontos em tempo real) |
| **Lag / f_release** | o pulso fica armado e só solta numa fração do downswing — é o que gera velocidade no swing |
