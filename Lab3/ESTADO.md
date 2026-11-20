# Lab 3 — ponto de situação (20/11) e o que falta

Visão de conjunto: tudo o que está feito, porquê, e o que falta, tarefa a
tarefa. O plano original está em `PLANO.md`; aqui só o estado.
Entrega: **18 dez 2026**.

Legenda: ✅ feito · 🟡 em parte · ❌ por fazer

## Resumo

| Tarefa | Dados | Código | Resultados | Relatório |
|---|---|---|---|---|
| 1 Modelos | — | — | — | ✅ escrita |
| 2 Calibração | 🟡 robô sim, mesa não | ✅ | 🟡 só com o robô | ❌ |
| 3 Inclinómetro | 🟡 robô sim, mão não | ✅ | ✅ com o robô | ❌ |
| 4 Giroscópio | 🟡 robô sim, mão não | ✅ | ✅ com o robô | ❌ |
| 5 Fusão (CI) | ✅ robô | ❌ | ❌ | ❌ (métodos escritos) |
| 6 Gestos | 🟡 robô sim, mão não | ❌ | ❌ | ❌ (métodos escritos) |
| Vídeos | 🟡 robô sim, mão não | — | — | ❌ |

**O que mais falta:** os ensaios à mão (A a F), o código das Tarefas 5 e 6, e
o relatório.

## Material e preparação ✅

- **Sketch próprio** (`src/arduino/imu_bruto/`): lê o chip diretamente, em
  bruto, ±4 g e ±500 °/s, 200 Hz, 500000 baud.
  *Porquê:* a biblioteca da cadeira arredonda o giroscópio a 1 °/s (mais do
  que o desvio a medir) e lê mal a temperatura abaixo de 25 °C.
  Verificado a 20/11: 200 Hz, 0 falhas.
- **Gravador** (`src/gravar.py`), **simulador** (`src/simular.py`) e análise
  validada contra a verdade do simulador antes de haver dados reais.
- **Suporte da IMU** (`modelos/suporte_imu/`, peça "1"): substitui um dedo da
  pinça; Uno nas torres de espaçadores, placa Grove apertada por 3 M3.
  Impresso a 16/11, montado e usado a 20/11.
  *Porquê:* o robô dá a verdade (orientação e posição) a cada 10 ms.
- **Ensaios do robô** (`src/robo/`): planeados e verificados no MuJoCo com o
  suporte montado; `sessao.py` corre-os por ordem, todos a partir da pose
  casa, com as mudanças de modo do controlador (que dão um solavanco) fora da
  gravação.
- **Sincronização IMU–robô** (`src/robo/verdade.py`): −5 a −10 ms em todos os
  ensaios de 20/11.

## Dados gravados (20/11)

| Ensaio | Gravações | Para quê |
|---|---|---|
| 0 (teste rápido, mesa) | 1 | taxa, ligação, ruído na mesa |
| poses (robô) | 2 | calibração e orientação da montagem; calibrar numa, verificar na outra |
| rotacoes (robô) | 1 | escala do giroscópio |
| inclinacoes (robô) | 2 | Tarefas 3 e 4 |
| abanar (robô) | 2 | inclinómetro com aceleração; Tarefa 5 |
| vaivem (robô) | 2 | critério do peso dinâmico (D3) |
| acenar (robô) | 2 | gesto da Tarefa 6 com verdade |
| deriva (robô) | 3 meia velocidade + 1 normal | posição, ensaio G |
| motores_ligados (robô parado) | 1 | vibração dos servos |
| mesa_robo (com e sem DC/DC) | 2 | a mesa do robô não vibra |

As gravações da 1.ª volta (antes das 11:36) têm o solavanco dentro dos troços
parados; a análise corta-o. Configuração do robô em
`data/robot_info_2026-11-20.txt`.

## Tarefa 1 — Modelos ✅

Escrita no relatório (`secoes/t1.tex`). Modelos com matriz 3×3 completa
(escala, eixos não perpendiculares e rotação da montagem) e desvio.

## Tarefa 2 — Calibração 🟡

**Feito:**
- Calibração com o robô (`src/t2_robo.py`): poses e rotações da 1.ª sessão,
  verificada na 2.ª sessão de poses.

  | | Acelerómetro | Giroscópio |
  |---|---|---|
  | Desvio | (−20,9; 4,0; 1,4) mg | (−0,31; −0,62; −0,31) °/s |
  | Escalas | 1,006 / 1,001 / 0,989 | 0,996 / 0,994 / 0,991 |
  | Rotação da placa | 4,6° | 4,3° |

  Verificação: ângulo à gravidade 4,6° → 0,8°; erro do giroscópio
  1,5 → 0,23 °/s. As duas sessões de poses dão a mesma calibração (a menos de
  1 mg, 0,1 % e 0,2°).
- Vibração dos servos medida: no braço parado o ruído é 4–9× (acc) e 6–20×
  (giro) o da mesa; a mesa do robô e o DC/DC não contam (`CONCEITOS.md`).

**Falta:**
- ❌ **Ensaio A/T** (25 min parado, kit frio): ruído, variância de Allan,
  desvio em função da temperatura. *Porquê:* no robô os servos vibram e não
  servem para o ruído; a temperatura só se vê começando frio.
- ❌ **B e B2** (seis posições e ao acaso, à mão): calibração clássica e por
  elipsoide, para comparar com a do robô. *Porquê:* a tabela de parâmetros
  pelos três métodos é um dos resultados pedidos.
- ❌ **C** (rotações à mão contra a régua): escala do giroscópio sem robô.
- ❌ Correr `t2_calibracao.py` com os dados reais (já testado no simulador).
- ❌ Filtragem (o enunciado sugere): que filtro, corte e efeito no ruído.
- ❌ Figuras 1 a 4 do relatório (|a| antes e depois, giroscópio contra robô,
  temperatura, Allan).

## Tarefas 3 e 4 — Inclinómetro e giroscópio 🟡

**Feito**, com a calibração do robô e a verdade do robô:

| Ensaio | Acelerómetro RMS (máx) | Giroscópio no fim | Sem calibrar |
|---|---|---|---|
| inclinacoes (2 sessões) | 1,7–1,8° (12–13°) | 2,6–2,7° em 110 s | 60–62° |
| abanar | 3,1° (17°) | 0,9° | 40° |
| acenar | 2,5° (14°) | 0,7° | 17° |

**Falta:**
- 🟡 `t3_t4_orientacao.py` procura troços parados pela IMU, o que no robô não
  encontra nenhum (vibração); passar a usá-los pelas juntas, como em
  `t2_robo.py`.
- ❌ **D, D2, E à mão** (inclinações com transferidor, abanar, mexer e pousar):
  os mesmos métodos sem robô, comparados com os ângulos ditos no vídeo.
- ❌ Figura 5 (varrimento de inclinação com verdade, os três métodos e o erro).

## Tarefa 5 — Fusão ❌

**Feito:** métodos escritos no relatório (`secoes/t5.tex`), incluindo os três
critérios do peso dinâmico e o ensaio D3. Dados do robô prontos (abanar,
vaivém e acenar em duas sessões: ajustar numa, avaliar na outra).

**Falta:**
- ❌ Código da CI com peso fixo: varrer o parâmetro, erro RMS contra a verdade
  do robô (o que o enunciado pede).
- ❌ O melhor peso fixo no ensaio lento e no com abanões. *Porquê:* mostrar
  que os mínimos caem em sítios diferentes, logo um valor fixo não chega.
- ❌ Peso dinâmico com os três critérios; D3 (vaivém) mostra a falha do
  critério |a| ≈ 1 g na horizontal.
- ❌ R ou peso com o ruído **do robô**, não da mesa (a vibração dos servos).
- ❌ Figura 6 (erro em função do parâmetro, mínimos, peso dinâmico).
- ❌ Extra: Kalman como comparação (a confirmar com o professor).

## Tarefa 6 — Gestos ❌

**Feito:** métodos no relatório (`secoes/t6.tex`), incluindo a correção da
deriva entre repousos. Dados do robô: acenar (2), deriva (meia velocidade e
normal).

**Falta:**
- ❌ **F1 e F2 à mão** (acenar e soco, 5 de cada). *Porquê:* o enunciado pede
  o gesto com o dispositivo na mão; o robô entra como validação.
- ❌ Código: orientação com o melhor parâmetro; trajetória por dupla
  integração; correção entre repousos (rampa de velocidade).
- ❌ Ensaio G: posição estimada contra a do robô e contra a previsão ½·b·t²,
  com e sem calibração e com e sem a correção.
- ❌ Comparar meia velocidade com velocidade normal no deriva (o mesmo
  percurso, erros diferentes).
- ❌ Figuras 7 e 8.

## Vídeos 🟡

- ✅ 21 vídeos de 20/11 em `video/media/ensaios_2026-11-20/` (fora do git,
  1,8 GB), cada um com o mesmo nome do ficheiro de dados do seu ensaio
  (`<data>_<hora>_<ensaio>.MOV`); 4 sem ensaio (tentativas falhadas e um clip
  solto). `correspondencia.csv` diz de que `IMG_xxxx` veio cada um. O
  `112128_deriva` foi filmado só em parte (32 de 56 s).
- ❌ Vídeos dos ensaios à mão (o enunciado exige-os na Tarefa 2).
- ❌ Ligações no relatório (`secoes/videos.tex`).

## Relatório ❌

Esqueleto no formato dos Labs 1 e 2, modelos e métodos escritos; 44 `\todo`
por preencher (t2: 11, t5: 8, t6: 7, comentário: 6, montagem: 5, t4: 3,
t3: 2, extra e vídeos: 1). `CONCEITOS.md` tem as explicações e os nomes para
escrever as partes de ruído, temperatura e vibração.

## Perguntas ao professor (em aberto)

- O filtro de Kalman é pedido ou é extra?
- Na Tarefa 6, espera só a orientação ou também a posição?
- "O melhor parâmetro" tem de ser fixo, ou pode ser um peso que se adapta?

## Ordem sugerida

1. Ensaios à mão: A/T primeiro (kit frio), depois B, B2, C, D, D2, E, F1, F2,
   com vídeo (`ROTEIRO_ROBO.md`, parte 2).
2. Calibração com os dados da mão e tabela dos três métodos (Tarefa 2).
3. Código da fusão (Tarefa 5) sobre os dados do robô.
4. Gestos e posição (Tarefa 6), robô e mão.
5. Figuras, relatório e vídeos.

Há uma marcação no robô a 26/11 (LSDC3, 10:00–11:30): pode servir para
repetir algum ensaio ou para os extras, se for preciso.
