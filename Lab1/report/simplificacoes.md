# Simplificações e impacto no realismo — Golfista V13

Tarefa 1 do enunciado: *"List any simplifications/assumptions made and their
potential impact in the realism of the simulation."*

Os números citados obtêm-se com `python src/swing.py` e `python src/perturbacoes.py`
(a partir de `Lab1/`). Referências "reais" são para um amador razoável com um
ferro 7.

## Resultado do modelo vs. realidade

| Grandeza | Modelo | Real (ferro 7) | |
|---|---|---|---|
| Velocidade da cabeça do taco | 37.2 m/s | 30–38 m/s | ✅ |
| Velocidade da bola | 45.6 m/s | ~45 m/s | ✅ |
| Smash factor (v_bola / v_taco) | 1.22 | 1.25–1.33 | ✅ |
| Duração do downswing | 0.30 s | 0.25–0.30 s | ✅ |
| Ângulo de ataque | −3.0° | ~−4° | ✅ |
| Loft dinâmico | 31.6° | ~25° | ❌ |
| Ângulo de lançamento | 31.7° | 16–20° | ❌ |
| Spin | 3100 rpm | ~7000 rpm | ❌ |
| Carry | 189 m | 140–150 m | ❌ |

A cinemática e a dinâmica do swing estão em gama realista. O que falha está
todo **depois** do impacto — na transferência para a bola e no voo —, e cada
desvio tem uma causa identificada abaixo.

---

## 1. Corpo

### 1.1 Só dois graus de liberdade: braço + pulso
Tudo o resto — pernas, ancas, tronco, cotovelos — está rígido na pose de endereço.

- **Porquê estes dois:** são o pêndulo duplo de Cochran & Stobbs (1968), o
  modelo canónico do swing. A primeira versão usava tronco + braços, mas esses
  dois tinham **correlação 0.905** na matriz de massa (eixos a 35° um do outro,
  ambos a rodar o taco sempre esticado): eram praticamente o mesmo movimento, e
  a velocidade ficava presa nos 25.7 m/s. Com braço + pulso a correlação desce
  para 0.57 e a velocidade sobe para 37.2 m/s.
- **Impacto:** perde-se a **sequência cinemática** real (ancas → tronco → braços
  → taco, cada segmento a atingir o pico de velocidade depois do anterior). No
  modelo, o motor do braço tem de fazer sozinho o trabalho que num corpo real
  vem das pernas e do tronco — por isso leva ±150 N·m, e não os ~50–100 N·m de
  um ombro isolado.

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
  aconteça cedo.

### 2.2 Trajetória imposta, não otimizada
O controlador segue uma trajetória escolhida à mão (aceleração constante, mesmo
T nas duas juntas, pulso a soltar a 65 % do downswing). Não é o swing ótimo —
é um swing plausível que garante que a cabeça passa na bola.

- **Impacto:** um golfista real não "sabe" a sua trajetória; ajusta-a por
  prática. O instante de libertação do pulso, sozinho, vale +39 % de velocidade
  (0.10 → 26.8 m/s, 0.65 → 37.2 m/s), o que mostra quanto o resultado depende
  de uma escolha que aqui é manual.

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
  pequeno. Um estudo de convergência levou o passo de 5e-4 s para **5e-5 s**:
  acima disso a velocidade da bola ainda dependia do passo.

### 3.2 O lançamento sai alto: 31.7° em vez de 16–20°
Duas causas, com pesos semelhantes, medidas no modelo.

**Causa A — loft dinâmico alto (31.6° vs ~25°).** No modelo o impacto dá-se
exatamente na pose de endereço (q = 0 nas duas juntas), pelo que a face chega
com a mesma inclinação que tinha em repouso. Um golfista real chega ao impacto
com as mãos **mais à frente** do que no endereço (inclinação da haste para o
alvo), o que fecha a face uns 6–8°. Com 2 dof não existe nenhum grau de
liberdade que incline a haste para a frente independentemente do resto.

**Causa B — a bola sai pela normal da face.** Numa bola real, o atrito faz a
bola "rolar para cima" da face durante o impacto; isso converte parte do loft
em **backspin** e baixa o lançamento para ~82 % do loft. A teoria para uma
esfera que agarra a face (loft 32°, coeficiente de restituição ~0.8) dá
**lançamento 26.3° e ~6300 rpm**. O modelo dá **31.7° e ~3100 rpm**:

| Atrito taco-bola μ | Lançamento | Lançamento / loft | Spin |
|---|---|---|---|
| 0.0 | 32.6° | 1.03 | 500 rpm |
| 0.4 (atual) | 31.7° | 1.00 | 3100 rpm |
| 1.2 | 33.6° | 1.06 | 4600 rpm |

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

### 4.1 Sem aerodinâmica
Não há arrasto nem sustentação (efeito Magnus do backspin).

- **Impacto:** a bola faz uma parábola exata. **189 m é exatamente o alcance
  de uma parábola** para 45.6 m/s a 31.7°. Uma bola real com essa velocidade
  voa 140–150 m: o arrasto tira muito mais distância do que o Magnus devolve.
  O carry do modelo **não deve ser comparado** com valores reais; a velocidade
  e o ângulo de lançamento sim.

### 4.2 Relvado plano e rígido
- **Impacto:** depois de aterrar, a bola rola demais: não há deformação da
  relva nem backspin a travá-la. Por isso a métrica usada é o **carry**
  (distância até à primeira aterragem), e não a distância total.

---

## 5. Perturbações (Tarefa 2)

### 5.1 Ruído no binário, com banda limitada
O "pulso a tremer" do enunciado é modelado como ruído gaussiano no binário da
junta, **mantido durante 1/10 s** (banda do tremor fisiológico, 8–12 Hz).

- **Porquê não ruído branco a cada passo:** com passo de 5e-5 s há ~6000 passos
  num downswing; amostras independentes cancelavam-se e o efeito medido era
  praticamente zero. Pior: o resultado dependia do passo de integração, o que
  tirava sentido ao teste.
- **Impacto:** o tremor real não é gaussiano nem de frequência fixa. O modelo
  captura a ordem de grandeza, não a forma.

### 5.2 A saturação esconde o ruído pequeno
O pulso está saturado 30 % do downswing. Ruído somado a um comando já no
limite é cortado — por isso σ = 3 N·m quase não se nota e o teste usa 10 N·m.

- **Leitura:** é um efeito real e interessante — um movimento feito no limite
  da força é mais repetível face a ruído no comando, porque a saturação o
  corta. Mas também significa que o teste só mostra efeitos acima de um certo
  nível de ruído.

### 5.3 O pulso é o grau de liberdade sensível
Com σ = 10 N·m: ruído no **pulso** → ~6 m de dispersão lateral e algumas bolas
"topadas" (lançamento negativo); ruído no **braço** → ~0.2 m de dispersão
lateral. O pulso controla a orientação da face no impacto; o braço controla
sobretudo a velocidade.

---

## 6. O que a V17 deve atacar, por ordem de impacto

1. **Aerodinâmica da bola** (arrasto + Magnus) — torna o carry comparável com a
   realidade. É só código Python no voo, não mexe no modelo do corpo.
2. **Inclinação da haste no impacto** — um 3.º dof, ou uma pose de impacto
   diferente da de endereço, para baixar o loft dinâmico (causa A de 3.2).
3. **Ombros articulados** — resolve o artefacto visual 1.2 e dá inércia
   variável.
4. **Rotação do tronco** — repõe a sequência cinemática (1.1, 1.3).
