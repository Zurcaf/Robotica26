# Bibliografia — Lab 1

Todas as entradas foram confirmadas numa fonte primária (página da editora, DOI,
catálogo ou documentação oficial) em outubro de 2026. Ao lado de cada uma está
**para que serve** no relatório, para que a citação fique ligada a um número ou
a uma decisão concreta.

## Robótica e controlo

- [1] J. S. Sequeira, *Introdução à Robótica*, Coleção Ensino da Ciência e da
  Tecnologia n.º 83. Lisboa: IST Press, 2024. ISBN 978-989-8481-99-3.
  — Livro recomendado na cadeira. **Cap. 5 "Dinâmica"**: §5.4 formulação
  lagrangiana, §5.5 estrutura das equações `M(q)q̈ + C(q,q̇)q̇ + g(q) = τ`
  (é o `M2 @ a_cmd + h2` do `swing.py`), §5.6 efeitos dissipativos (o
  `damping` das juntas). **Cap. 7 "Controlo"**: §7.5 controlo PID, §7.6
  controlo não linear de robôs (linearização por realimentação = binário
  calculado). **Cap. 8** §8.4 geração de trajetórias (a referência de jerk
  mínimo / parábola do downswing).
- [2] B. Siciliano, L. Sciavicco, L. Villani e G. Oriolo, *Robotics: Modelling,
  Planning and Control*. London: Springer, 2009. ISBN 978-1-84628-641-4.
  doi:10.1007/978-1-84628-642-1.
  — Cap. 7 "Dynamics" (pp. 247–302) e Cap. 8 "Motion Control" (pp. 303–361);
  **§8.5.2 "Inverse Dynamics Control"** é exatamente o controlador usado.
- [3] J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 3.ª ed.
  Upper Saddle River, NJ: Pearson Prentice Hall, 2005. ISBN 978-0-201-54361-2.
  (4.ª ed.: Pearson, 2018, ISBN 978-0-13-348979-8.)
  — Cap. 6 "Manipulator dynamics"; Cap. 9 "Linear control" (secção
  *control-law partitioning*, a ideia de separar feedforward do modelo e PD
  do erro); Cap. 10 "Nonlinear control" (aplicação ao manipulador completo).

## MuJoCo

- [4] E. Todorov, T. Erez e Y. Tassa, "MuJoCo: A physics engine for model-based
  control," em *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems
  (IROS)*, Vilamoura, 2012, pp. 5026–5033. doi:10.1109/IROS.2012.6386109.
  — O simulador; o modelo de contacto mole (`solref`/`solimp`) e a razão de
  não existir coeficiente de restituição.
- [5] MuJoCo Documentation, "Computation — Fluid forces," Google DeepMind,
  <https://mujoco.readthedocs.io/en/stable/computation/fluid.html>, e "XML
  Reference — geom/fluidcoef",
  <https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom-fluidcoef>.
  Acedido em out. 2026.
  — Fundamenta três pontos do §4.1 do relatório:
  (a) o arrasto rombo do modelo de elipsoide é `f_D = −ρ [C_blunt A_proj +
  C_slender (A_max − A_proj)] |v| v`, **sem o fator ½** — por isso
  `fluidcoef[0] = 0.14` dá Cd efetivo 0.28;
  (b) o Magnus é `f_M = C_M ρ V ω × v`, linear na rotação (a limitação
  discutida em §4.1);
  (c) o modelo "inertia-based", usado quando a geom não declara
  `fluidshape`, assume uma caixa de inércia equivalente com arrasto
  `f_D,i = −2 ρ r_j r_k |v_i| v_i` — a origem dos 18.7 N·m no pulso. Valores
  por omissão de `fluidcoef`: `0.5 0.25 1.5 1.0 1.0` (rombo, esguio, angular,
  Kutta, Magnus). O modelo inclui massa adicionada (Tuckerman, 1925), que não
  pode ser desligada — é a razão de `Magnus = 0` não anular a sustentação.

## Física do golfe

- [6] A. Cochran e J. Stobbs, *The Search for the Perfect Swing*. London:
  Heinemann, 1968. Reimpressão: Chicago: Triumph Books, 2005,
  ISBN 978-1-57243-729-6.
  — O modelo do swing como **pêndulo duplo** (braço + taco) e o papel da
  libertação tardia do pulso. Justifica a escolha dos 2 dof (§1.1).
- [7] T. P. Jorgensen, *The Physics of Golf*, 2.ª ed. New York: Springer
  (AIP Press), 1999. ISBN 978-0-387-98691-3.
  — Tratamento quantitativo do pêndulo duplo, do "lag" e do voo da bola.
- [8] A. R. Penner, "The physics of golf," *Rep. Prog. Phys.*, vol. 66, n.º 2,
  pp. 131–171, 2003. doi:10.1088/0034-4885/66/2/202.
  — Revisão: swing, impacto (smash factor, loft dinâmico, spin ≈ f(loft)) e
  aerodinâmica. Referência geral para a tabela "modelo vs. realidade".
- [9] A. R. Penner, "The physics of golf: The optimum loft of a driver," *Am.
  J. Phys.*, vol. 69, n.º 5, pp. 563–568, 2001. doi:10.1119/1.1344164.
  — Relação entre loft, lançamento e spin no impacto (§3.2: lançamento ≈ 0.8 ×
  loft numa bola que agarra a face).
- [10] A. R. Penner, "The run of a golf ball," *Can. J. Phys.*, vol. 80,
  n.º 8, pp. 931–940, 2002. doi:10.1139/p02-035.
  — Ressalto e rolamento depois de aterrar; justifica usar o **carry** e não a
  distância total (§4.2).

## Aerodinâmica da bola

- [11] P. W. Bearman e J. K. Harvey, "Golf ball aerodynamics," *Aeronaut. Q.*,
  vol. 27, n.º 2, pp. 112–122, 1976. doi:10.1017/S0001925900007617.
  — Medições em túnel de vento (Re 40k–240k, S 0.02–0.3): Cd ≈ 0.27–0.32 e CL
  a subir de ≈0.08 a ≈0.25 com S. A S ≈ 0.15: **Cd ≈ 0.28–0.30, CL ≈
  0.15–0.20**. Os valores efetivos do modelo (0.28 / 0.20) caem dentro.
- [12] A. J. Smits e D. R. Smith, "A new aerodynamic model of a golf ball in
  flight," em *Science and Golf II*, A. J. Cochran e M. R. Farrally (eds.).
  London: E & FN Spon, 1994, pp. 340–347.
  — Correlações `Cd = 0.24 + 0.18 S + 0.06 sin[π(Re − 90000)/200000]` e
  `CL = 0.54 S^0.4` (CL ≈ 0.25 a S = 0.15); decaimento da rotação `dω/dt =
  −2.0e−5 (v²/R²) S`, i.e. **≈4 %/s a 45 m/s**, proporcional a v². Mostra que
  o CL real **satura** com S (expoente 0.4), ao contrário do Magnus linear do
  MuJoCo — a causa de a "tacada perfeita" a 6700 rpm voar alto demais (§4.1).
- [13] G. Tavares, K. Shannon e T. Melvin, "Golf ball spin decay model based
  on radar measurements," em *Science and Golf III*, M. R. Farrally e A. J.
  Cochran (eds.). Champaign, IL: Human Kinetics, 1999, pp. 464–472.
  — Medições radar: `dω/dt = −R ρ A C_M v²/I`, C_M ≈ 0.012 S; ≈5 %/s à
  velocidade de lançamento. Com [12], justifica os 3.5 %/s do modelo
  (`fluidcoef[2] = 0.1`) como "ordem de grandeza certa".
- [14] R. D. Mehta, "Aerodynamics of sports balls," *Annu. Rev. Fluid Mech.*,
  vol. 17, pp. 151–189, 1985. doi:10.1146/annurev.fl.17.010185.001055.
  — Revisão: papel das covinhas na transição da camada limite (porque o Cd de
  uma bola de golfe é ~0.25 e não ~0.5 como numa esfera lisa a este Re).
- [15] S. F. Hoerner, *Fluid-Dynamic Drag*. Bakersfield, CA: publicado pelo
  autor, 1965. (Cap. III, "Pressure drag".) Alternativa de acesso fácil:
  F. M. White, *Fluid Mechanics*, 7.ª ed. New York: McGraw-Hill, 2011, Tab. 7.3.
  — Cd de corpos rombos: placa quadrada normal ao escoamento 1.17, disco 1.1,
  cilindro em escoamento cruzado (Re 10⁴–2×10⁵) ≈ 1.2. É o **Cd ≈ 1.1 da
  estimativa à mão** do arrasto na cabeça e na haste (§4.1,
  `teste_aerodinamica.py`).
- [16] USGA e R&A, "Pendulum Test Technical Description" e "TPX3007 Initial
  Velocity Test Protocol", <https://www.usga.org/equipment-standards>.
  — Limites regulamentares: COR da **face do taco** 0.822 (+0.008 de
  tolerância = 0.830); para a **bola** o limite é de velocidade inicial
  (≤ 250 ft/s + 2 %). Nota: o 0.83 citado no XML é o limite do taco, não da
  bola; a frase no comentário do `solref` deve dizer "limite da face".

## Tremor e ruído motor

- [17] C. M. Harris e D. M. Wolpert, "Signal-dependent noise determines motor
  planning," *Nature*, vol. 394, pp. 780–784, 1998. doi:10.1038/29528.
  — O ruído motor humano cresce com a amplitude do comando. Justifica (a) a
  escolha de ruído no **binário** e não na posição, e (b) a observação de que
  σ constante é uma simplificação (§5.1).
- [18] R. J. Elble e W. C. Koller, *Tremor*. Baltimore: Johns Hopkins
  University Press, 1990. ISBN 978-0-8018-4024-1. E: G. Deuschl, P. Bain e
  M. Brin, "Consensus statement of the Movement Disorder Society on tremor,"
  *Mov. Disord.*, vol. 13, supl. 3, pp. 2–23, 1998. doi:10.1002/mds.870131303.
  — Tremor fisiológico: banda larga (3–30 Hz) com a componente **reforçada**
  centrada em 8–12 Hz. Justifica a largura de banda de 10 Hz do ruído (§5.1).
  Cuidado na redação: "8–12 Hz" é a banda do tremor fisiológico *reforçado*.

## Biomecânica do swing

- [19] T. Jorgensen, "On the dynamics of the swing of a golf club," *Am. J.
  Phys.*, vol. 38, n.º 5, pp. 644–651, 1970. doi:10.1119/1.1976419.
  — O artigo clássico do pêndulo duplo: dois segmentos, binário no ombro e
  pulso passivo/atrasado. É o modelo do `golfer_v13.xml` (§1.1).
- [20] W. M. Pickering e G. T. Vickers, "On the double pendulum model of the
  golf swing," *Sports Eng.*, vol. 2, n.º 3, pp. 161–172, 1999.
  doi:10.1046/j.1460-2687.1999.00028.x.
  — Mostra analiticamente que atrasar a libertação do pulso ("late hit")
  aumenta a velocidade da cabeça. Justifica o `f_release = 0.65` e os +31 %
  medidos (§2.2).
- [21] E. J. Sprigings e R. J. Neal, "An insight into the importance of wrist
  torque in driving the golfball: A simulation study," *J. Appl. Biomech.*,
  vol. 16, n.º 4, pp. 356–366, 2000. doi:10.1123/jab.16.4.356.
  — Simulação com binário ativo no pulso: o pulso contribui pouco em binário
  mas muito em velocidade. Justifica o motor do pulso fraco (±40 N·m) com
  libertação controlada (§2.1).
- [22] R. D. Milne e J. P. Davis, "The role of the shaft in the golf swing,"
  *J. Biomech.*, vol. 25, n.º 9, pp. 975–983, 1992.
  doi:10.1016/0021-9290(92)90033-W.
  — A flexão da haste contribui pouco para a velocidade: justifica a haste
  rígida.
- [23] S. M. Nesbit, "A three dimensional kinematic and kinetic study of the
  golf swing," *J. Sports Sci. Med.*, vol. 4, n.º 4, pp. 499–519, 2005.
  PMCID PMC3899667.
  — 85 amadores com driver: velocidade da cabeça 39–52 m/s (média 46.4);
  **binário máximo aplicado pelas mãos ao taco ≈ 31 N·m** (componente "alpha",
  no plano do swing). É a referência golf-específica para o `ctrlrange` do
  pulso. Nesbit nota ainda que a libertação tardia resulta de binário
  positivo mantido, não de "travar" o pulso.
- [24] P. A. Hume, J. Keogh e D. Reid, "The role of biomechanics in maximising
  distance and accuracy of golf shots," *Sports Med.*, vol. 35, n.º 5,
  pp. 429–449, 2005. doi:10.2165/00007256-200535050-00005.
  — Revisão: sequência cinemática proximal→distal e X-factor, i.e. o que o
  modelo de 2 dof **não** tem (§1.1, §1.3).
- [25] T. Harbo, J. Brincks e H. Andersen, "Maximal isokinetic and isometric
  muscle strength of major muscle groups related to age, body mass, height,
  and sex in 178 healthy subjects," *Eur. J. Appl. Physiol.*, vol. 112, n.º 1,
  pp. 267–275, 2012. doi:10.1007/s00421-011-1975-3.
  — Binários máximos (homens, Tab. 3): flexão do pulso 22–25 N·m; flexão do
  cotovelo ~51 N·m; abdução/adução do ombro 60–83 N·m. Fundamenta os ±40 N·m
  do pulso (pico, com margem) e torna explícito que os **±150 N·m do "ombro"
  não são um ombro**: representam toda a cadeia pernas + tronco + braços
  (§1.1, §2.1). Não foi encontrada fonte para flexão/extensão do ombro.

## Dados de referência de um ferro 7

- [26] TrackMan A/S, "TrackMan Average Tour Stats" (tabela 2015) e "New PGA &
  LPGA Tour Averages" (2 maio 2024),
  <https://www.trackman.com/blog/introducing-updated-tour-averages>.
  — PGA Tour, ferro 7 (tabela 2015): taco 40.2 m/s, bola 53.6 m/s, smash
  1.33, ataque −4.3°, lançamento 16.3°, spin 7097 rpm, altura máx. 29 m,
  descida 50°, carry 157 m. LPGA Tour: taco 34.0 m/s, bola 44.7 m/s, smash
  1.38, lançamento 19.0°, spin 6699 rpm, carry 129 m. (A tabela de 2024 dá
  PGA 41.1 / 55.0 m/s, 7124 rpm, 161 m — citar a versão usada.) São as gamas
  da tabela "modelo vs. realidade" e do `trajetoria.py`.
- [27] R. D. Grober, "An accelerometer based instrumentation of the golf
  club: Comparative analysis of golf swings," arXiv:1001.0761, 2010.
  — Duração do downswing: profissionais 258 ± 8 ms, amador de hcp 10
  385 ± 20 ms, hcp 25 433 ± 18 ms (driver). Os 0.30 s do modelo são de
  jogador bom. *Preprint, não revisto por pares.*
- [28] Golf Monthly, "How far does the average PGA Tour player hit his
  7-iron" (cita dados TrackMan de amadores),
  <https://www.golfmonthly.com/features/how-far-does-the-average-pga-tour-player-hit-his-7-iron>.
  — Amador médio (hcp ~14): taco ~35.8 m/s, carry ~133 m. Fonte secundária;
  usar só para a nota "amador" da tabela.

## Notas sobre o que NÃO foi confirmado

- "Binário de flexão do ombro 100–150 N·m": sem fonte. O texto do relatório
  passou a dizer que os ±150 N·m representam a cadeia inteira.
- COR 0.83: é o limite da **face do taco**, não da bola ([16]).
- Hoerner [15]: a existência do livro está confirmada, mas não a página/figura
  dos Cd; os valores 1.1–1.2 são os tabelados em White, Tab. 7.3.
