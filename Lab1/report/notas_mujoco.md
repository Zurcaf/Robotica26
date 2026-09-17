# Notas — como funciona o MuJoCo (e o que fizemos nos pêndulos)

Notas de aprendizagem para o grupo. Ler com os ficheiros `models/00_pendulum.xml`,
`models/01_pendulum_ball.xml`, `src/run_pendulum.py` e `src/run_pendulum_ball.py` abertos ao lado.

---

## 0. O que é um dof (grau de liberdade)

Pensa numa **porta**. Está presa à parede por dobradiças. Para descrever *como ela está* neste
momento, quanta informação precisas? **Um número**: o ângulo de abertura. 0° fechada, 90° aberta.
A posição de cada ponto da porta fica determinada por esse ângulo. A porta tem **1 dof**.
É o nosso pêndulo.

Agora uma **bola no ar**. Para dizer onde está e como está orientada precisas de x, y, z (onde)
+ 3 ângulos (como está rodada). **6 dof.** É o `freejoint` da bola.

Um **braço humano** simplificado: ombro roda (1), cotovelo dobra (1), pulso dobra (1). **3 dof.**
O taco na mão não acrescenta nenhum — está agarrado, move-se com o pulso.

> **dof = quantos números independentes preciso para descrever a configuração completa do sistema.**
> Cada junta livre acrescenta os seus; cada coisa soldada não acrescenta nenhum.

Porque é que o prof insiste nisto:

- Cada dof é uma **equação de movimento** que o simulador resolve. Mais dof = mais complexo.
- Cada dof é um sítio onde tens de decidir **motor, limites, atrito** (hints 6 e 7).
- Cada dof é um sítio onde pode haver **perturbação**. "Pulso a tremer" = ruído no dof do pulso.
  A Tarefa 2 pede ruído *em cada um* — menos dof, menos testes.

Golfista V13: tronco (1) + ombro (1) = **2 dof** + bola (6). V17: + pulso = **3 dof**.

Truque para fixar: **dof = número de sliders no painel "Joint" do viewer.** No pêndulo há um: `hinge`.

---

## 1. O MuJoCo em 3 ideias

**Ideia 1 — Modelo vs. Dados.**
- **Modelo** (`MjModel`): descrição *estática* — corpos, massas, ligações. Vem do XML. Nunca muda.
- **Dados** (`MjData`): o *estado* neste instante — posições, velocidades, forças, tempo. Muda a cada passo.

Analogia: o modelo é a planta do mecanismo; os dados são a fotografia dele num instante.

**Ideia 2 — A simulação é um loop de passos.** Não há "play". Chamas `mj_step(model, data)` e ele
avança `timestep` segundos (pusemos 0.001 s). 1000 chamadas = 1 s simulado. Tu controlas o loop.

**Ideia 3 — Árvore cinemática.** O mundo é a raiz, cada `<body>` está pendurado no pai, e uma
`<joint>` dentro de um body diz *como esse body se pode mover relativamente ao pai*. Sem joint,
está soldado. É a lógica das cadeias cinemáticas das teóricas.

---

## 2. O XML (formato MJCF) — `models/00_pendulum.xml`

```xml
<mujoco model="pendulum">
  <option gravity="0 0 -9.81" timestep="0.001"/>
```
Opções globais. Z é "para cima". `timestep` = quanto avança cada `mj_step`. Mais pequeno = mais preciso, mais lento.

```xml
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.8 0.9 0.8 1"/>
```
`worldbody` é a raiz — o referencial do mundo, fixo. A `light` é só para o viewer. Um `geom` é uma
**forma geométrica** com dois fins: colisão e visual. `plane` = chão infinito.

```xml
    <body name="pivot" pos="0 0 1.5">
```
Um **corpo rígido** com o seu referencial em (0, 0, 1.5) relativamente ao pai (o mundo). O `pos` do
body é a origem do **referencial desse corpo** — hint 3 do prof.

```xml
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.0"/>
```
A junta. `hinge` = dobradiça. `axis="0 1 0"` = roda em torno de Y. Está dentro do `pivot`, logo diz
"o pivot roda em torno de Y relativamente ao mundo". **1 dof.** `damping` = atrito viscoso (torque
proporcional à velocidade); zero para o período bater com a teoria.

```xml
      <geom name="rod" type="capsule" fromto="0 0 0  0 0 -1.0" size="0.02" mass="0.01" .../>
      <geom name="bob" type="sphere" pos="0 0 -1.0" size="0.08" mass="1.0" .../>
```
Dois geoms **dentro do body pivot** — rodam com ele. Coordenadas sempre relativas ao body.

> Quando dás `mass` a um geom, o MuJoCo **calcula a inércia automaticamente** a partir da forma.
> O body soma massa e inércia de todos os seus geoms. Podes também escrever `<inertial>` à mão
> (na V17 provavelmente vamos querer).

```xml
  <visual><global azimuth="120" elevation="-20"/></visual>
  <statistic center="0 0 1" extent="2.5"/>
```
Só define a câmara inicial do viewer. Não afeta a física.

---

## 3. Pêndulo + bola — `models/01_pendulum_ball.xml`

```xml
  <default>
    <geom solref="0.002 1" solimp="0.9 0.95 0.001"/>
  </default>
```
`default` aplica atributos a todos os geoms. `solref`/`solimp` controlam a **rigidez dos contactos** —
o MuJoCo não faz contactos perfeitamente rígidos, são molas muito duras. O primeiro valor do `solref`
é a constante de tempo dessa mola (2 ms = pancada seca). Sítio para mexer nos testes da Tarefa 2.

```xml
    <geom name="floor" type="plane" ... friction="0.8 0.005 0.0001"/>
```
Três atritos: deslizamento, torção, rolamento. O de rolamento faz a bola parar em vez de rolar para sempre.

```xml
    <body name="ball" pos="0.1 0 0.0213">
      <freejoint name="ball_free"/>
      <geom name="ball_geom" type="sphere" size="0.0213" mass="0.0459"/>
    </body>
```
`freejoint` = livre no espaço: 3 translações + 3 rotações = **6 dof**. Sem joint seria uma esfera
soldada ao mundo. Valores reais: 45.9 g, raio 21.3 mm, `pos` z = raio para ficar pousada.

Total de dof: 1 (hinge) + 6 (bola) = 7.

---

## 4. O Python — `src/run_pendulum.py`

Bibliotecas:
- `mujoco` — o simulador. O único que interessa aprender.
- `numpy` — arrays numéricos; o MuJoCo devolve tudo como arrays numpy.
- `math`, `sys` — Python standard.

```python
model = mujoco.MjModel.from_xml_path(MODEL)
data = mujoco.MjData(model)
```
Lê o XML → modelo. Cria o estado a partir do modelo.

```python
mujoco.mj_resetData(model, data)
data.qpos[0] = theta0
```
Reset. `qpos` é o **vetor de posições generalizadas** — uma entrada por dof. No pêndulo tem
tamanho 1: o ângulo da hinge em radianos. Escrever aqui = colocar o pêndulo nesse ângulo.
(Há também `qvel`, as velocidades.)

```python
while data.time < t_max:
    mujoco.mj_step(model, data)
    cur = data.qpos[0]
```
O loop de simulação. `data.time` avança 0.001 a cada step.

O resto é matemática: detetar cruzamentos por zero, a diferença entre eles é o período, comparar
com `2π√(L/g)`. Deu 0.03–0.1% de erro — prova de que a física está bem (hint 1).

```python
if "--view" in sys.argv:
    mujoco.viewer.launch(model, data)
```
Janela interativa; o viewer faz os `mj_step` sozinho.

---

## 5. `src/run_pendulum_ball.py`

```python
ball = model.body("ball").id
```
Nome no XML → índice inteiro que os arrays usam.

```python
v = np.linalg.norm(data.cvel[ball][3:])
```
`data.cvel` = velocidade de cada body, 6 números `[ωx ωy ωz vx vy vz]` (angular primeiro, linear
depois — convenção do MuJoCo). `[3:]` apanha a linear; `norm` dá o módulo.

```python
return data.xpos[ball].copy(), v_max
```
`data.xpos` = posição (x, y, z) de cada body no mundo. O `.copy()` é porque o array é uma vista
para a memória interna do MuJoCo — sem copiar, mudava por baixo de ti no próximo step.

O `qpos` da bola tem 7 entradas (3 posição + 4 quaternion) para 6 dof; é mais fácil ler `xpos`.

---

## 6. Os três nomes que se usam a toda a hora

| No XML | No Python | O que é |
|---|---|---|
| `<joint>` | `data.qpos`, `data.qvel` | estado de cada dof |
| `<body>` | `data.xpos`, `data.cvel` | posição/velocidade cartesiana de cada corpo |
| `<motor>` (próximo passo) | `data.ctrl` | comando que dás a cada atuador |

O golfista vai acrescentar `<actuator><motor joint="..."/>` no XML e `data.ctrl[i] = torque` no
Python — é assim que o "jogador" faz força. As perturbações da Tarefa 2 são literalmente
`data.ctrl[i] = torque + ruído`.

---

## 7. Viewer — cheat sheet

Abrir (caminho **absoluto**, o viewer perde a pasta atual no Mac):
```bash
.venv/bin/python -m mujoco.viewer --mjcf="$PWD/models/00_pendulum.xml"
```
Ou abrir vazio e arrastar o XML para a janela. Não usar `mjpython` (partido nesta versão).

| Tecla / botão | Faz |
|---|---|
| **Space** | play / pause |
| **Backspace** | reset ao estado inicial |
| **Ctrl+L** ou *Simulation → Reload* | recarrega o XML depois de o editares |
| scroll / arrastar botão esquerdo | zoom / rodar câmara |
| duplo-clique num corpo + Ctrl+arrastar | aplica uma força (útil para perturbações à mão) |
| painel direito → **Joint** | sliders, um por dof — mexer o robot à mão |
| painel esquerdo → *Rendering → Frame → Body* | desenha os referenciais de cada corpo (hint 3) |

O viewer **não é um editor**: alteras o XML no VS Code, guardas, Ctrl+L no viewer.

Exercício: abre o pêndulo, expande *Joint*, vê o slider `hinge` — esse é o dof. Depois muda a
`mass` da esfera no XML, Ctrl+L, e vê o que muda. Aprende-se mais a partir coisas do que a ler.
