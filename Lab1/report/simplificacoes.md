# Simplificações e impacto no realismo — Golfista V13

Tarefa 1 do enunciado: *"List any simplifications/assumptions made and their
potential impact in the realism of the simulation."*

Os números citados obtêm-se com `python src/swing.py`, `python src/perturbacoes.py`,
`python src/teste_aerodinamica.py` e `python src/trajetoria.py` (a partir de
`Lab1/`). As gamas "reais" são as médias publicadas para um ferro 7, do LPGA
Tour ao PGA Tour (TrackMan [26]; ver `bibliografia.md`), salvo indicação em
contrário.

## Resultado do modelo vs. realidade

| Grandeza | Modelo | Real (ferro 7) | |
|---|---|---|---|
| Velocidade da cabeça do taco | 38.8 m/s | 34.0–40.2 m/s (amador ~36) | ✅ |
| Velocidade da bola | 47.2 m/s | 44.7–53.6 m/s | ✅ |
| Smash factor (v_bola / v_taco) | 1.22 | 1.33–1.38 | ⚠️ |
| Duração do downswing | 0.30 s | 0.26 s (pro) – 0.39 s (hcp 10) | ✅ |
| Ângulo de ataque | −3.2° | −4.3° | ✅ |
| Loft dinâmico | 31.4° | ~25° | ❌ |
| Ângulo de lançamento | 31.3° | 16–19° | ❌ |
| Spin | 3200 rpm | 6700–7100 rpm | ❌ |
| Carry | 151 m | 129–157 m | ✅ (ver 4.1) |
| Altura máxima | 39 m | 24–29 m | ❌ |
| Ângulo de descida | 49° | 47–50° | ✅ |

A cinemática e a dinâmica do swing estão em gama realista. O que falha está
todo **depois** do impacto — na transferência para a bola e no voo —, e cada
desvio tem uma causa identificada abaixo. O carry cai na gama real, mas por
**compensação de erros** (lançamento alto × spin baixo), não porque o voo esteja
certo: a altura máxima denuncia-o (ver 4.1).

---

## 1. Corpo

### 1.1 Só dois graus de liberdade: braço + pulso
Tudo o resto — pernas, ancas, tronco, cotovelos — está rígido na pose de endereço.

- **Porquê estes dois:** são o pêndulo duplo de Cochran & Stobbs (1968), o
  modelo canónico do swing. A primeira versão usava tronco + braços, mas esses
  dois tinham **correlação 0.905** na matriz de massa (eixos a 35° um do outro,
  ambos a rodar o taco sempre esticado): eram praticamente o mesmo movimento, e
  a velocidade ficava presa nos ~26 m/s. Com braço + pulso a correlação desce
  para 0.57 e a velocidade sobe para 38.8 m/s.
- **Impacto:** perde-se a **sequência cinemática** real (ancas → tronco → braços
  → taco, cada segmento a atingir o pico de velocidade depois do anterior). No
  modelo, o motor do braço tem de fazer sozinho o trabalho que num corpo real
  vem das pernas e do tronco — por isso leva ±150 N·m, quando um ombro isolado
  produz 60–85 N·m em abdução/adução [25]. O valor não é o de uma articulação:
  é o da cadeia inteira.

### 1.2 Os dois braços são um só corpo rígido
Não há ombros articulados: os braços rodam em bloco em torno do centro do peito.

- **Impacto visual:** a meio do swing os ombros deslizam alguns cm face ao
  tronco. Na primeira versão o eixo passava pelo ombro *esquerdo* e o braço
  direito chegava a descolar; passar o eixo para o centro do peito reduziu o
  problema e, como efeito lateral, baixou a inércia do braço 27 % (de 2.43 para
  1.78 kg·m²).
- **Impacto dinâmico:** num swing real o braço direito dobra e desdobra, o que
  muda a inércia ao longo do swing. Aqui a inércia do braço é constante.
- **V17:** ombros como rótulas, um de cada lado.

### 1.3 Tronco rígido — não há "X-factor"
A separação entre a rotação das ancas e a dos ombros, que os golfistas usam
para armazenar energia elástica, não existe.

- **Impacto:** o swing não pode beneficiar do ciclo alongamento-encurtamento
  muscular. É compensado, em parte, pelo binário alto do motor do braço (1.1).

### 1.4 Corpo só visual, sem contactos
Só a cabeça do taco colide, e só com a bola.

- **Impacto:** o taco atravessaria as pernas se o swing o mandasse para lá, e a
  sola da cabeça entra uns mm no relvado sem resistência. Num ferro real isto é
  em parte verdade (tira-se "divot"), mas perde-se a travagem que o relvado faz
  na cabeça.

### 1.5 Massas e inércias aproximadas
Tabelas antropométricas genéricas (braço ~5 % de 80 kg), geometria de cápsulas.
As inércias são calculadas pelo MuJoCo a partir das formas.

- **Impacto:** a massa real dos braços e a sua distribuição variam muito entre
  pessoas; os binários necessários escalam diretamente com elas.

---

## 2. Atuação e controlo

### 2.1 Motores de binário ideais, com saturação
Cada junta tem um motor que aplica diretamente o binário pedido, cortado em
±150 N·m (braço) e ±40 N·m (pulso).

- **Impacto:** um músculo real não é uma fonte de binário. A força que produz
  cai com a velocidade de contração (relação força-velocidade de Hill) e demora
  dezenas de ms a ativar. O modelo dá o binário máximo instantaneamente e a
  qualquer velocidade — é otimista precisamente na fase mais rápida do swing.
- O pulso leva pouco binário porque, num swing real, a libertação é em boa
  parte **passiva** (força centrífuga); o jogador sobretudo resiste a que ela
  aconteça cedo [20], [21]. Os ±40 N·m estão acima do máximo isométrico de
  flexão do pulso (22–25 N·m [25]) mas na ordem do binário que as mãos aplicam
  de facto ao taco num swing medido (≈31 N·m [23]).

### 2.2 Trajetória imposta, não otimizada
O controlador segue uma trajetória escolhida à mão (aceleração constante, mesmo
T nas duas juntas, pulso a soltar a 65 % do downswing). Não é o swing ótimo —
é um swing plausível que garante que a cabeça passa na bola.

- **Impacto:** um golfista real não "sabe" a sua trajetória; ajusta-a por
  prática. O instante de libertação do pulso, sozinho, vale +31 % de velocidade
  (soltar a 10 % do downswing → 29.6 m/s; a 65 % → 38.8 m/s), o que mostra
  quanto o resultado depende de uma escolha que aqui é manual.

### 2.3 Controlo por binário calculado (modelo perfeito da própria dinâmica)
`tau = M(q)·a_cmd + h(q, v)` usa a matriz de massa e as forças de Coriolis e
gravidade **exatas** do próprio modelo.

- **Impacto:** o controlador conhece a dinâmica do corpo sem erro. Um humano
  tem um modelo interno aproximado e atrasos sensoriais de ~100 ms. O modelo
  segue a trajetória melhor do que qualquer pessoa conseguiria.

---

## 3. Impacto taco-bola

### 3.1 Contacto mole com restituição calibrada
O MuJoCo não tem coeficiente de restituição: os contactos são molas amortecidas.
A restituição obtém-se deixando o contacto **sub-amortecido**
(`solref="0.0004 0.15"`), calibrado por varrimento para smash factor ~1.2.

- **Impacto:** o parâmetro não tem significado físico direto; foi ajustado para
  acertar num resultado. Com amortecimento crítico (o valor por omissão) o
  choque era totalmente inelástico e o smash factor dava 0.81.
- **Cuidado numérico:** contactos sub-amortecidos só convergem com passo
  pequeno. Com 5e-5 s em toda a simulação a velocidade da bola ainda saltava
  44.7–49.0 m/s (±5 %) com mudanças de 1 ms na duração do downswing, consoante o
  ponto do passo em que o contacto começava. O `run_swing` usa agora **5e-6 s
  enquanto a cabeça está a menos de 15 cm da bola** e 5e-5 s no resto, o que
  reduz a variação a ~1 % sem tornar a simulação mais lenta.
- **Cuidado na medição:** a velocidade da cabeça tem de ser lida **antes** do
  passo em que o contacto é detetado (depois dele já inclui parte do impulso), e
  a velocidade da bola **também** no passo em que o contacto acaba (o estado
  usado pelo MuJoCo para detetar contactos é o do início do passo). Antes destas
  correções a velocidade do taco estava subestimada em ~4 % (37.2 em vez de
  38.8 m/s) e a da bola podia ficar 20 % abaixo do valor real.

### 3.2 O lançamento sai alto: 31.3° em vez de 16–19°
Duas causas, com pesos semelhantes, medidas no modelo.

**Causa A — loft dinâmico alto (31.4° vs ~25°).** No modelo o impacto dá-se
exatamente na pose de endereço (q = 0 nas duas juntas), pelo que a face chega
com a mesma inclinação que tinha em repouso. Um golfista real chega ao impacto
com as mãos **mais à frente** do que no endereço (inclinação da haste para o
alvo), o que fecha a face uns 6–8°. Com 2 dof não existe nenhum grau de
liberdade que incline a haste para a frente independentemente do resto.

**Causa B — a bola sai pela normal da face.** Numa bola real, o atrito faz a
bola "rolar para cima" da face durante o impacto; isso converte parte do loft
em **backspin** e baixa o lançamento para ~82 % do loft. A teoria para uma
esfera que agarra a face (loft 32°, coeficiente de restituição ~0.8) dá
**lançamento 26.3° e ~6300 rpm**. O modelo dá **31.3° e ~3200 rpm**:

| Atrito taco-bola μ | Lançamento | Lançamento / loft | Spin |
|---|---|---|---|
| 0.0 | 32.5° | 1.04 | 600 rpm |
| 0.4 (atual) | 31.3° | 1.00 | 3200 rpm |
| 1.2 | 33.1° | 1.06 | 4450 rpm |

Aumentar o atrito gera mais spin mas **não baixa o lançamento**. Testaram-se
também o cone de atrito elíptico e o `noslip_iterations` do solver (feito
precisamente para eliminar deslizamento em contactos moles): nenhum mudou o
resultado mais de 0.2°. Conclui-se que o modelo de contacto mole do MuJoCo não
reproduz, num impacto de ~0.4 ms, a física da bola a rolar pela face.

- **Impacto:** as duas causas juntas explicam a diferença: 25° × 0.82 ≈ 20.5°,
  que é o valor real.

### 3.3 Bola rígida, sem deformação
Uma bola de golfe comprime-se ~1/3 do diâmetro no impacto de um driver.

- **Impacto:** contribui para 3.2 (a compressão é parte do mecanismo que gera
  spin) e faz com que o tempo de contacto seja um parâmetro (`solref`) e não uma
  consequência do material.

---

## 4. Voo da bola

### 4.1 Aerodinâmica com coeficientes constantes
O ar está ligado para todo o modelo (`density="1.204"`, `viscosity="1.81e-5"`
no `<option>`). A bola tem arrasto e sustentação de Magnus com o modelo de
elipsoide do MuJoCo; o braço e o taco também sentem o ar, cada geom como um
elipsoide equivalente. Os coeficientes **efetivos** foram medidos em voo livre
por `src/teste_aerodinamica.py`, porque o MuJoCo não usa o fator ½ da definição
clássica e os valores do `fluidcoef` não são os Cd/CL da literatura:

| | Modelo (medido) | Literatura (bola de golfe) |
|---|---|---|
| Cd | 0.28 | ~0.22–0.30 |
| CL a S = rω/v = 0.15 | 0.20 | ~0.15–0.25 |
| Decaimento da rotação | 3.5 %/s | poucos %/s |

- **Validação cruzada:** a bola lançada sozinha com as condições do swing
  (47.2 m/s, 31.3°, 3200 rpm) aterra a 151.3 m — o mesmo valor do swing
  completo. Sem Magnus daria 121 m e no vácuo 201 m.
- **Ar no braço e no taco:** o arrasto tira **1.3 %** à velocidade da cabeça
  (39.3 → 38.8 m/s) e vale no máximo 3.1 N·m no pulso — coerente com a
  estimativa à mão (cabeça de 12 cm² + haste, Cd ≈ 1.1: ~3.0 N·m). O modelo por
  omissão do MuJoCo quando a geom não declara `fluidshape` ("caixa de inércia")
  dava **18.7 N·m**, seis vezes mais, e tirava 9 % à velocidade; foi por isso
  que se declarou `fluidshape="ellipsoid"` em todas as geoms. O controlador
  **não recupera** esta perda mesmo que conheça o arrasto (`comp_fluid=True`
  muda a velocidade em <0.5 %): os motores estão saturados precisamente no fim
  do downswing, onde o arrasto é maior.
- **Impacto — coeficientes constantes:** o Magnus do MuJoCo é linear na rotação
  (F ∝ ρ V ω × v) e o Cd não depende do número de Reynolds. Numa bola real o CL
  satura a rotações altas. O `trajetoria.py --perfeita` mostra-o: lançada no
  mesmo modelo com as condições médias do LPGA Tour (46.5 m/s, 19°, 6700 rpm),
  a bola voa **162 m com 44 m de altura**, contra 129 m e 24 m publicados. O
  modelo está calibrado para a rotação **baixa** do nosso swing (3200 rpm), não
  para as 7000 rpm reais.
- **Impacto — o carry "bate certo" por compensação:** o lançamento alto (31°)
  e o spin baixo dão um voo que aterra na gama real (151 m vs 129–157 m), mas
  com uma altura máxima de 39 m (real: 24–29 m) e um tempo de voo de 6.8 s. É
  um resultado certo pelas razões erradas, e deve ser lido como tal.

### 4.2 Relvado plano e rígido
- **Impacto:** depois de aterrar, a bola rola demais: não há deformação da
  relva nem backspin a travá-la. Por isso a métrica usada é o **carry**
  (distância até à primeira aterragem), e não a distância total.

---

## 5. Perturbações (Tarefa 2)

### 5.1 Ruído no binário, com banda limitada
O "pulso a tremer" do enunciado é modelado como ruído gaussiano no binário da
junta, **mantido durante 1/10 s** (banda do tremor fisiológico reforçado,
8–12 Hz [18]). Pôr o ruído no binário, e não na posição, segue a evidência de
que o ruído motor humano é "signal-dependent" — cresce com o comando [17].

- **Porquê não ruído branco a cada passo:** com passo de 5e-5 s há ~6000 passos
  num downswing; amostras independentes cancelavam-se e o efeito medido era
  praticamente zero. Pior: o resultado dependia do passo de integração, o que
  tirava sentido ao teste.
- **Impacto:** o tremor real não é gaussiano nem de frequência fixa, e a sua
  amplitude cresce com o binário comandado [17]; aqui σ é constante. O modelo
  captura a ordem de grandeza, não a forma.
- **Cuidado:** σ = 10 N·m é 25 % do limite do motor do pulso (±40) mas só
  6.7 % do do braço (±150). A comparação entre juntas abaixo é em valor
  absoluto, não em fração da capacidade de cada motor.

### 5.2 A saturação esconde o ruído pequeno
O pulso está saturado 30 % do downswing. Ruído somado a um comando já no
limite é cortado — por isso σ = 3 N·m quase não se nota e o teste usa 10 N·m.

- **Leitura:** é um efeito real e interessante — um movimento feito no limite
  da força é mais repetível face a ruído no comando, porque a saturação o
  corta. Mas também significa que o teste só mostra efeitos acima de um certo
  nível de ruído.

### 5.3 O pulso é o grau de liberdade sensível
20 swings por junta, σ = 10 N·m, sementes 0–19 (`python src/perturbacoes.py
--junta wrist|shoulder`). Todos acertaram na bola.

| | braço | pulso |
|---|---|---|
| v cabeça do taco | 38.8 ± 0.1 m/s | 39.0 ± 0.3 m/s |
| ângulo de lançamento | 31.5 ± 0.2° | 30.7 ± 7.7° (mín 2°) |
| carry | 151.6 ± 0.7 m | 142.5 ± 32 m (mín 4 m) |
| desvio lateral | 0.06 ± 0.29 m | 3.3 ± 6.1 m (máx 16.6 m) |

- **Braço:** quase nada. O motor tem 15× mais capacidade do que o ruído, a
  inércia do braço é alta e o PD corrige o erro de posição antes do impacto.
- **Pulso:** a distribuição é **bimodal**, e a média ± desvio padrão descreve-a
  mal. 15 das 20 bolas caem a menos de 2 m do nominal; 4 vão 10–17 m para a
  direita (a face chega aberta, lançamento 35–40°); 1 é "topada" (lançamento
  2°, 4 m de carry). O que o pulso controla é a **orientação da face** no
  impacto, e essa é a variável a que o resultado é mais sensível.
- **Nota numérica importante:** na versão anterior do modelo o ruído no braço
  dava ±16 m de carry. Essa dispersão **não era do ruído**: era o contacto
  taco-bola mal resolvido (ver 3.1), que fazia a velocidade da bola variar
  ±5 % com mudanças de 1 ms no instante do impacto. Com o passo fino no
  contacto, a dispersão do braço cai para ±0.7 m. Um teste de perturbações só
  vale depois de garantir que a simulação nominal é numericamente estável.
- **Limitação do controlador:** o binário calculado conhece a dinâmica exata e
  tem ganhos altos; rejeita o ruído melhor do que um humano com atrasos de
  ~100 ms. A dispersão medida é um **limite inferior** da real.

---

## 6. O que a V17 deve atacar, por ordem de impacto

1. **Inclinação da haste no impacto** — um 3.º dof, ou uma pose de impacto
   diferente da de endereço, para baixar o loft dinâmico (causa A de 3.2). É o
   que mais afasta o voo do real (altura máxima, spin).
2. **Magnus dependente da rotação** — CL(S) com saturação, em vez do
   coeficiente constante do MuJoCo (4.1). Exige calcular a força em Python e
   aplicá-la em `xfrc_applied`, porque o modelo de fluido do MuJoCo não é
   configurável a esse nível.
3. **Ombros articulados** — resolve o artefacto visual 1.2 e dá inércia
   variável.
4. **Rotação do tronco** — repõe a sequência cinemática (1.1, 1.3).
