# Lab 3 — plano de ataque

Prazo: 18 dez 2026. Escrito a 13 nov, antes de qualquer ida ao laboratório;
atualizado a 14 nov com o software pronto (ver `src/README.md`) e o roteiro da
primeira ida (`ROTEIRO_17NOV.md`).
O que está marcado **(a confirmar)** é suposição minha, ainda não verificada no
material.

## O que o enunciado pede

Uma IMU Grove (acelerómetro + giroscópio de 3 eixos; o magnetómetro não se
usa) ligada a um Arduino Uno, que manda os dados por porta série para o
portátil. Seis tarefas, respondidas uma a uma no relatório:

| # | Pede | O que é preciso para responder |
|---|---|---|
| 1 | Modelos matemáticos do acelerómetro e do giroscópio; dizer que parâmetros se estimam | só papel |
| 2 | Ensaios para estimar esses parâmetros, **com vídeo**; comentar. Dica: filtrar os dados | ensaios A, B, C |
| 3 | Inclinómetro só com o acelerómetro | ensaio D |
| 4 | Orientação só com o giroscópio | ensaios C, D, E |
| 5 | Fundir 3 e 4, por exemplo com Covariance Intersection (CI); gráficos para vários valores do parâmetro | ensaio D (o mesmo ficheiro serve) |
| 6 | Com o melhor parâmetro: gesto de acenar e gesto de soco, devagar; resultados claros | ensaio F |

O título é "trajectory estimation": na tarefa 6 espera-se a orientação ao longo
do gesto e, tanto quanto der, a trajetória da mão.

## O que já se sabe do material (lido dos scripts e da biblioteca)

- O kit é um **ICM-20600** (Grove, I2C, endereço 0x69), lido com a biblioteca
  da Seeed no sketch da cadeira, que envia `tempo ax ay az wx wy wz` em mg e
  °/s a **9600 baud, ~15 amostras/s**.
- **A biblioteca arredonda**: devolve inteiros em mg e °/s. O sketch deixa o
  giroscópio a ±2000 °/s, logo a resolução é 1 °/s, maior do que o desvio
  que queremos medir. E `getTemperature()` lê o registo sem sinal: abaixo de
  25 °C dá lixo.
- **O nosso sketch** (`src/arduino/imu_bruto/`) lê os registos diretamente,
  só com `Wire`: contagens em bruto e temperatura, **±4 g e ±500 °/s, 200 Hz a
  taxa fixa, 500000 baud** (0 % de erro com o cristal do Uno; a linha ocupa
  ~25 % da porta). Compila para o Uno (16 % da memória). Não precisa de
  bibliotecas instaladas.
- **Guardar no Arduino não dá**: o Uno tem 2 KB de RAM (o sketch deixa 1,6 KB
  livres, ~1 s de dados) e 1 KB de EEPROM. O sketch da cadeira tem código
  para cartão SD comentado, mas o kit não tem módulo SD. Com 500000 baud a
  porta série chega e sobra.
- O tempo vem do Arduino (µs); o gravador junta a hora do computador a cada
  linha, para sincronizar com o robô.

## Modelos (Tarefa 1), para saber o que medir

Acelerómetro, parado ou em movimento:

    a_medido = S_a · R^T · (a − g) + b_a + ruído

Giroscópio:

    w_medido = S_g · w + b_g + ruído

Parâmetros a estimar: os desvios `b_a` e `b_g` (3 + 3), os fatores de escala
`S_a` e `S_g` (3 + 3, diagonais; o desalinhamento entre eixos só se os dados o
mostrarem), e as variâncias do ruído (3 + 3), que são também o que a CI
precisa na Tarefa 5.

## Ensaios a gravar no laboratório

Cada ensaio é um ficheiro de dados e um vídeo. Nome sugerido:
`data/AAAA-MM-DD_<letra>_<descrição>_<n>.csv`.

| | Ensaio | Como | Duração | Para quê |
|---|---|---|---|---|
| 0 | Taxa de amostragem | sensor parado; 30 s com o sketch da cadeira (9600 baud) e 30 s com o nosso (500000 baud, 200 Hz) | 1 min | confirmar a taxa e que não há falhas; mostrar a diferença de resolução |
| A | Parado, longo | sensor pousado na mesa, sem tocar | 10 a 30 min | desvio e ruído do giroscópio; ruído do acelerómetro; deriva ao aquecer (ligar e gravar logo) |
| B | Seis posições | sensor preso a uma caixa ou esquadro; cada face para baixo, 30 s parado em cada (±x, ±y, ±z) | 5 min | desvio e escala do acelerómetro: desvio = média das duas leituras opostas; escala = metade da diferença, a dividir por g |
| C | Rotação conhecida | rodar a caixa encostada a uma régua: 90° e 360° em torno de cada eixo, nos dois sentidos, 3 vezes | 10 min | escala do giroscópio: ângulo integrado a dividir pelo ângulo real |
| B2 | Posições ao acaso | 15 a 20 posições bem espalhadas, parado 10 s em cada, sem alinhar nada | 5 min | o mesmo que B por outro método: procurar o desvio e a escala que fazem o tamanho da aceleração dar 1 g em todas. Comparar com B e com o robô. As mesmas paragens dão o desvio do giroscópio |
| T | Temperatura | sensor parado, a começar frio (acabado de ligar, ou vindo do frigorífico num saco) até estabilizar, a gravar a temperatura do chip | 20 a 30 min | desvio do giroscópio e do acelerómetro em função da temperatura; pode ser o mesmo registo do ensaio A |
| D | Inclinações conhecidas | inclinar a 0, 15, 30, 45, 60, 90° em rolamento e em arfagem, parando 5 s em cada, e voltar a 0; referência: transferidor ou inclinómetro do telemóvel | 5 min, 3 vezes | inclinómetro (T3), giroscópio (T4) e fusão (T5) sobre os mesmos dados, com valor de referência |
| D2 | Inclinar a abanar | o mesmo que D, mas a meio dar pequenos abanões laterais | 2 min | mostrar onde o acelerómetro falha e o giroscópio não: é o que justifica a fusão |
| D3 | Vaivém (no robô) | sem rodar, vaivém de ±50 mm de lado e na vertical, a 0,1 g e 0,2 g | 1 min | mostra quando o teste "|a| ≈ 1 g" falha: na horizontal |a| quase não muda (0,1 g de lado dá 1,005 g) mas a inclinação erra 5,7°; na vertical |a| muda 10 % e o teste apanha |
| E | Parado depois de mexer | mexer 20 s e pousar na posição inicial; 2 min parado | 3 min, 3 vezes | deriva do giroscópio; erro ao voltar ao ponto de partida |
| F1 | Acenar | cotovelo apoiado, antebraço a oscilar devagar; começar e acabar parado 3 s; régua visível no vídeo | 20 s, 5 vezes | Tarefa 6 (i) |
| F2 | Soco | mão a avançar devagar ao longo de uma régua de comprimento conhecido e a voltar; começar e acabar parado 3 s | 20 s, 5 vezes | Tarefa 6 (ii) |

Em F1 e F2 o movimento tem geometria conhecida (comprimento da régua, raio do
antebraço), o que dá um valor de referência sem equipamento extra. Começar e
acabar parado permite corrigir a deriva da integração.

Levar: caixa ou esquadro rígido, fita-cola, transferidor, régua, tripé ou
apoio para o telemóvel, **adaptador USB-C → USB-A** (o Arduino Uno liga por
USB-A), portátil com o Arduino IDE e as bibliotecas do sketch já instaladas.

Notas sobre a calibração:

- O método das posições ao acaso (B2) corrige o desvio e a escala do
  acelerómetro sem precisar de alinhar o sensor, mas não diz como o chip está
  montado em relação ao suporte: isso só o robô dá.
- O desvio do giroscópio muda cada vez que se liga o sensor e com a
  temperatura (fichas técnicas: da ordem de 0,02 °/s por °C no ICM-20600, a
  confirmar na ficha da TDK). Mede-se no início de cada sessão, com o sensor
  já quente. Todos os ensaios começam e acabam com 5 a 10 s parado.
- Gravar sempre os dados em bruto e, se a biblioteca a der, a temperatura do
  chip **(a confirmar no sketch)**.

## O robô como referência (proposta do professor)

O Lite 6 do Lab 2 sabe a sua pose a cada 10 ms. Com a IMU presa à pinça,
substitui o transferidor e a régua por uma referência contínua no tempo.
Os ensaios estão planeados e verificados no MuJoCo com o suporte e o kit
(`src/robo/`, imagens em `resultados/robo/`):

| Ensaio | Duração | O que faz | Para quê |
|---|---|---|---|
| poses | 279 s | 17 orientações paradas 8 s (5 faces + 12 ao acaso); a face com a pinça para cima não tem solução | B e B2 no robô; rotação entre o chip e o flange |
| rotacoes | 115 s | J6 ±270° e J5 ±45° a velocidade constante (30, 90, 140 °/s; J5 até 120) | C: escala e desalinhamento do giroscópio |
| inclinacoes | 103 s | rolamento e arfagem de 0 a 90° em passos de 15°, parado 5 s, a rodar em torno da própria IMU | D |
| abanar | 47 s | arfagem de 30° e 60°, com abanões de ±10 mm a 2 Hz (0,16 g) | D2 |
| vaivem | 50 s | ±50 mm de lado e na vertical, 0,1 g e 0,2 g | D3 |
| deriva | 25 s | soco de 300 mm em 2 s e um quadrado de 150 × 200 mm, parado no início e no fim | G |
| acenar | 18 s | ±30° em torno de um "cotovelo" 150 mm abaixo, 0,5 Hz | F1 com verdade |

Todos cumprem: juntas a ≥ 5° dos limites, ≤ 80 % das velocidades e
acelerações máximas, flange ≤ 400 mm/s, folga ≥ 20 mm no MuJoCo (a mais
pequena, 25 mm, é o corpo da pinça ao elo 4, cujo invólucro convexo é
conservador). Corre-se como no Lab 2, uma pose de juntas a cada 10 ms
(`set_servo_angle_j`), a gravar a IMU ao mesmo tempo:

    .venv/bin/python src/robo/verificar.py             # verifica todos
    .venv/bin/python src/robo/verificar.py acenar --ver  # janela MuJoCo
    .venv/bin/python src/robo/correr.py acenar --go     # no robô

A orientação do chip no flange (`R_F_IMU` em `src/robo/geometria.py`) é uma
hipótese até ao ensaio de poses; medir no laboratório o kit e a posição do
chip e corrigir `P_F_IMU`.

Reservas: a tarefa 6 pede o gesto "holding the device in your hand", por isso
os gestos à mão têm de existir e o robô entra como validação do método; o
ensaio A faz-se na mesa com o robô desligado (os motores vibram), e repete-se
com o robô parado a segurar a IMU para medir o ruído que ele acrescenta; a
referência é a pose que o controlador indica.

Sincronização: os dois registos (robô e IMU) são gravados no mesmo portátil,
com a hora do computador. O relógio do Arduino passa para essa hora por uma
reta, e o desfasamento fino é o que sobrepõe |ω| do robô e do giroscópio
(`src/robo/verdade.py`). Testado com atrasos inventados (relógio 120 ms
desviado, USB 2 a 15 ms, juntas 10 ms): a verdade fica a < 0,13° em 99 % das
amostras.

**Suporte:** `modelos/suporte_imu/` (substitui um dedo da pinça; o professor
autorizou; usa os próprios parafusos do dedo, M2,5 × 4, e os M3 do kit). Medidas da pinça em
`modelos/gripper_lite/MEDIDAS.md`.

### O que se ganha em fazer os ensaios no robô

- **Verdade conhecida**: o método das posições ao acaso (B2) funciona às cegas;
  no robô sabe-se a orientação de cada posição e verifica-se se o método deu a
  resposta certa.
- **Montagem**: só com orientações conhecidas se tira a rotação entre o chip e
  o suporte.
- **Repetibilidade**: um script percorre as mesmas posições três vezes; sai a
  calibração e a dispersão entre repetições, que é a barra de erro.
- **Cobertura**: as posições escolhem-se para ficarem bem espalhadas.

Ficam na mesa: A (ruído; os motores vibram mesmo parados), T (temperatura) e,
à mão, os gestos da tarefa 6.

Cuidados com a referência:

1. A mesa pode não estar nivelada: o robô sabe a orientação em relação à sua
   base, não à gravidade. Essa inclinação é a mesma em todas as posições, por
   isso estima-se junto com a calibração.
2. "Parado" no robô não é tão parado como na mesa: medir primeiro o ruído com
   o robô a segurar a posição.
3. A referência é a pose que o controlador indica.

## Ensaio G: a posição deriva (resultado a mostrar)

Um erro constante `b` na aceleração dá um erro de posição de `½·b·t²`:
10 mg por corrigir são 44 cm ao fim de 3 s; 1° de erro na inclinação são
77 cm (a gravidade mal subtraída deixa 0,17 m/s²). A previsão desenha-se com
os valores de `b` da tarefa 2, antes de medir.

1. Parado na mesa (o ensaio A): integrar duas vezes; a posição verdadeira é
   zero, tudo o que aparece é erro. Comparar com `½·b·t²`.
2. No robô, com trajetória conhecida: posição estimada contra a verdadeira.
3. O mesmo com e sem calibração, e com e sem pôr a velocidade a zero quando o
   sensor pára.
4. **Extra da tarefa 6: correção da deriva entre repousos.** O robô (e o
   gesto à mão) começa e acaba parado, logo a velocidade verdadeira é zero
   nas duas pontas. A velocidade integrada no fim não dá zero: esse valor é
   erro puro. Com um desvio constante o erro de velocidade é uma rampa de 0
   até b·T; subtrai-se a rampa e integra-se outra vez. Fica só o erro que
   muda durante o movimento (inclinação mal estimada, ruído). Figura: posição
   do robô, IMU sem correção e IMU corrigida. Custa umas linhas e usa os
   dados que já se gravam.

## Peso da fusão: fixo e dinâmico (tarefas 5 e 6)

O enunciado fala em "o melhor parâmetro", que sugere um valor fixo. Faz-se
isso e mais:

1. Peso fixo, vários valores (o pedido).
2. O melhor peso fixo no ensaio lento (D) e no ensaio com abanões (D2): os
   mínimos caem em sítios diferentes, logo um valor fixo não chega.
3. Peso dinâmico: em cada instante, o que minimiza a incerteza final, com a
   incerteza do acelerómetro a subir conforme três critérios:

   | Critério | Apanha | Falha |
   |---|---|---|
   | \|a\| longe de 1 g | acelerações verticais e fortes | acelerações horizontais pequenas: 0,1 g de lado dá \|a\| = 1,005 g e 5,7° de erro |
   | rotação rápida (\|ω\|) | gestos com rotação | translações sem rodar |
   | diferença entre o acelerómetro e a previsão do giroscópio | qualquer aceleração que desvie a vertical medida | demora a reagir se o giroscópio já tiver derivado |

   O ensaio D3 (vaivém) mostra a falha do primeiro na horizontal; o terceiro
   é o que um filtro de Kalman faz com a inovação.
4. Tarefa 6 com os dois: o melhor fixo e o dinâmico. O acenar testa a
   orientação; o soco testa a posição e a resistência da fusão à aceleração.

Perguntar ao professor se na tarefa 6 espera só a orientação ou também a
posição.

## Extra: filtro de Kalman, como comparação

O enunciado pede a fusão com CI ("for example"); o Kalman foi referido pelo
professor na aula e fica como extra, a confirmar com ele.

- Tarefas 3 e 4 são as duas metades de um Kalman: o giroscópio faz a previsão
  (rápida, deriva) e o acelerómetro a correção (lenta, não deriva); as
  variâncias da tarefa 2 dão os pesos.
- A CI não assume que os erros das duas fontes são independentes; o Kalman
  assume, e fica confiante de mais quando o erro do acelerómetro dura várias
  amostras (desvio mal calibrado, mão a acelerar).
- A mostrar, nos mesmos dados e contra a referência do robô: só giroscópio, só
  acelerómetro, CI com o melhor ω, e Kalman com o desvio do giroscópio no
  estado. Uma figura e uma tabela, meia página.
- No ensaio G: a posição com um Kalman que recebe "velocidade zero" quando o
  sensor pára, ou a pose do robô de vez em quando.

## Extra: controlar o robô com a mão

Demonstração da tarefa 5 em tempo real. Kit preso às costas da mão com
`modelos/placa_mao/`.

1. Primeiro no MuJoCo, com o modelo do Lite 6 do Lab 2: sem risco.
2. Depois no robô, muito limitado: poucas juntas, gamas pequenas, velocidade
   cortada, filtro, e uma tecla que tem de estar premida para ele se mexer.

Modos: a pinça copia a inclinação da mão; e "inclinar para mover", como um
joystick. Seguir a posição XYZ da mão diretamente não funciona (ensaio G), e
mostrar isso faz parte do vídeo.

## Antes de ir ao laboratório (feito a 14 nov)

A ideia é a do Lab 2: preparar e testar tudo em simulação, e ir ao laboratório
só para gravar. Tudo em `src/` (ver `src/README.md`):

1. **Gravador** (`gravar.py`): lê a porta série, grava CSV com o tempo do
   Arduino e a hora do computador, mostra a taxa, as falhas e |a|. Testado
   com um Arduino falso numa porta virtual (deteta amostras perdidas e linhas
   partidas).
2. **Simulador** (`simular.py`): nove cenários à mão e sete do robô, com
   desvio, escala, eixos não perpendiculares, ruído, deriva do desvio e
   temperatura conhecidos.
3. **Análise** das tarefas 2 a 4 (`t2_calibracao.py`, `t3_t4_orientacao.py`),
   verificada no simulador: recupera o desvio do acelerómetro a < 1 mg, as
   escalas a < 0,1 %, o desvio e o ruído do giroscópio.
4. **Sketch** a 500000 baud, compilado.
5. **Ensaios do robô** planeados e verificados no MuJoCo (secção acima).

Do simulador já sai uma conclusão para o ensaio T: em 10 min a aquecer 6 °C,
a deriva lenta do desvio confunde-se com o efeito da temperatura (sem ela, o
declive sai certo). O ensaio T precisa de uma variação maior, com o sensor a
vir do frio.

## Figuras do relatório (6 páginas: oito figuras no máximo)

Cada figura responde a uma pergunta. As marcadas com ★ são as indispensáveis.

| # | Tarefa | Figura | O que mostra | Ensaio |
|---|---|---|---|---|
| 1 ★ | 2 | Módulo da aceleração em cada posição, antes e depois de calibrar | a calibração funciona: os pontos passam a cair em 1 g | B, B2 |
| 2 ★ | 2 | Velocidade medida pelo giroscópio contra a imposta pelo robô, por eixo | a escala é o declive, o desvio é a ordenada na origem | C |
| 3 | 2 | Desvio do giroscópio em função da temperatura | quanto muda ao aquecer | T |
| 4 | 2 | Desvio de Allan dos dois sensores (ou sinal em bruto e filtrado, se faltar espaço) | o ruído e o efeito de filtrar, como o enunciado sugere | A |
| 5 ★ | 3, 4, 5 | Um varrimento de inclinação: ângulo verdadeiro (robô), acelerómetro, giroscópio e fusão; por baixo, o erro de cada um | cada método e onde falha, nos mesmos dados; inclui o troço com abanões | D, D2 |
| 6 ★ | 5 | Erro (RMS) da fusão em função do parâmetro da CI, para o ensaio lento e para o ensaio com abanões, com os mínimos marcados; por baixo, o peso dinâmico ao longo do tempo | o que o enunciado pede (o efeito do parâmetro), que o melhor valor depende do movimento, e o peso a adaptar-se sozinho | D, D2 |
| 7 ★ | 6 | Acenar e soco à mão: orientação ao longo do tempo e trajetória estimada, com fotogramas do vídeo | os dois gestos pedidos | F |
| 8 ★ | 6 | Erro de posição ao longo do tempo no robô, com a previsão ½·b·t² | até onde se pode confiar na trajetória, previsto contra medido | G |

Tabelas: (i) parâmetros estimados (desvio, escala, ruído) com a dispersão de 3
repetições, pelos três métodos (seis posições, posições ao acaso, robô);
(ii) erro RMS de orientação de cada método (acelerómetro, giroscópio, CI,
e Kalman se entrar).

Para poupar espaço: as tarefas 3, 4 e 5 partilham a figura 5; a figura 3 ou a
4 podem ir para o vídeo se não couberem. Vídeos de todos os ensaios da tarefa
2 (o enunciado exige-os), com ligação no relatório.

## Riscos conhecidos

- **A porta a 500000 baud não funcionar** (algum clone do Uno): baixar
  `BAUD` para 250000 ou 115200 no sketch e `RATE_HZ` para 100; o gravador
  aceita `--baud`.
- **Posição por dupla integração deriva em segundos.** A trajetória da tarefa
  6 só é credível em gestos curtos, com paragem no início e no fim.
- **Guinada**: o acelerómetro não a vê. A fusão só corrige rolamento e
  arfagem; a guinada fica só com o giroscópio, e deriva.
- **Vídeos**: o enunciado exige-os na Tarefa 2. Filmar todos os ensaios.

## Calendário proposto

| Quando | O quê |
|---|---|
| 14 nov | software, simulador e ensaios do robô verificados no MuJoCo (feito) |
| seg 17 nov | 1.ª ida: ensaios à mão (0, A/T, B, B2, C, D, D2, E, F1, F2) e confirmar no robô o que falta para o suporte (`ROTEIRO_17NOV.md`) |
| sex 21 nov | imprimir o suporte e a placa da mão na Composites |
| 23–29 nov | 2.ª ida: suporte na pinça e os sete ensaios do robô; tarefa 5 (CI) com os dados reais |
| 30 nov–6 dez | gestos com a placa da mão; o que faltar; extras |
| 7–13 dez | relatório |
| 14–18 dez | revisão e entrega |
