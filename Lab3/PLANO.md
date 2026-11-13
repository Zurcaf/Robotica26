# Lab 3 — plano de ataque

Prazo: 18 dez 2026. Escrito a 13 nov, antes de qualquer ida ao laboratório.
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

## O que já se sabe do material (lido dos scripts da cadeira)

- O Arduino envia uma linha por amostra: `tempo ax ay az wx wy wz`, com a
  aceleração em **mg** e a velocidade angular em **graus/s**.
- **A porta série está a 9600 baud.** Cada linha tem ~64 caracteres, o que dá
  no máximo **~15 amostras por segundo**. É pouco para integrar um soco. O
  enunciado deixa alterar o sketch: passar para 115200 baud deve dar mais de
  100 Hz **(a confirmar no laboratório; é a primeira coisa a medir)**.
- O sketch escolhe a gama do acelerómetro com `setAccScaleRange(0x00)` e deixa
  a do giroscópio por omissão. Para gestos lentos convém a gama mais pequena
  dos dois, que dá mais resolução **(a confirmar que valores são na biblioteca)**.
- O tempo vem do Arduino em microssegundos: usa-se esse, não o relógio do PC.
- O script Python da cadeira só imprime no ecrã, com a porta `COM3` (Windows).
  Precisamos de um que grave para ficheiro e funcione em Mac (`/dev/cu.usbmodem*`).

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
| 0 | Taxa de amostragem | sensor parado; 30 s a 9600 baud e 30 s a 115200 | 1 min | saber a que frequência amostramos e se há falhas |
| A | Parado, longo | sensor pousado na mesa, sem tocar | 10 a 30 min | desvio e ruído do giroscópio; ruído do acelerómetro; deriva ao aquecer (ligar e gravar logo) |
| B | Seis posições | sensor preso a uma caixa ou esquadro; cada face para baixo, 30 s parado em cada (±x, ±y, ±z) | 5 min | desvio e escala do acelerómetro: desvio = média das duas leituras opostas; escala = metade da diferença, a dividir por g |
| C | Rotação conhecida | rodar a caixa encostada a uma régua: 90° e 360° em torno de cada eixo, nos dois sentidos, 3 vezes | 10 min | escala do giroscópio: ângulo integrado a dividir pelo ângulo real |
| D | Inclinações conhecidas | inclinar a 0, 15, 30, 45, 60, 90° em rolamento e em arfagem, parando 5 s em cada, e voltar a 0; referência: transferidor ou inclinómetro do telemóvel | 5 min, 3 vezes | inclinómetro (T3), giroscópio (T4) e fusão (T5) sobre os mesmos dados, com valor de referência |
| D2 | Inclinar a abanar | o mesmo que D, mas a meio dar pequenos abanões laterais | 2 min | mostrar onde o acelerómetro falha e o giroscópio não: é o que justifica a fusão |
| E | Parado depois de mexer | mexer 20 s e pousar na posição inicial; 2 min parado | 3 min, 3 vezes | deriva do giroscópio; erro ao voltar ao ponto de partida |
| F1 | Acenar | cotovelo apoiado, antebraço a oscilar devagar; começar e acabar parado 3 s; régua visível no vídeo | 20 s, 5 vezes | Tarefa 6 (i) |
| F2 | Soco | mão a avançar devagar ao longo de uma régua de comprimento conhecido e a voltar; começar e acabar parado 3 s | 20 s, 5 vezes | Tarefa 6 (ii) |

Em F1 e F2 o movimento tem geometria conhecida (comprimento da régua, raio do
antebraço), o que dá um valor de referência sem equipamento extra. Começar e
acabar parado permite corrigir a deriva da integração.

Levar: caixa ou esquadro rígido, fita-cola, transferidor, régua, tripé ou
apoio para o telemóvel, **adaptador USB-C → USB-A** (o Arduino Uno liga por
USB-A), portátil com o Arduino IDE e as bibliotecas do sketch já instaladas.

## O robô como referência (proposta do professor)

O Lite 6 do Lab 2 sabe a sua pose a cada 10 ms e já temos o código que o move
e a grava. Com a IMU presa à pinça, substitui o transferidor e a régua por uma
referência contínua no tempo.

| Ensaio | Versão com o robô |
|---|---|
| B | 12 a 24 orientações exatas, parada 10 s em cada: desvio, escala e desalinhamento do acelerómetro |
| C | uma junta a rodar a velocidade constante (10, 30, 60 °/s), nos dois sentidos: escala do giroscópio |
| D | varrimento de inclinação com o ângulo verdadeiro em cada instante |
| D2 | inclinar e transladar ao mesmo tempo, com aceleração conhecida |
| F | "acenar" e "soco" desenhados nas juntas, como o swing do Lab 2, com posição e orientação verdadeiras |
| G (novo) | deriva da posição: ver abaixo |

Reservas: a tarefa 6 pede o gesto "holding the device in your hand", por isso
os gestos à mão têm de existir e o robô entra como validação do método; o
ensaio A faz-se na mesa com o robô desligado (os motores vibram), e repete-se
com o robô parado a segurar a IMU para medir o ruído que ele acrescenta; a
referência é a pose que o controlador indica.

Sincronização: os dois registos (robô e IMU) são gravados no mesmo portátil,
com o mesmo relógio, e cada ensaio começa com um movimento brusco e curto que
se vê nos dois.

**Suporte:** `modelos/suporte_imu/` (substitui um dedo da pinça; precisa da
autorização do professor e de 2 parafusos M2,5 × 5). Medidas da pinça em
`modelos/gripper_lite/MEDIDAS.md`.

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

## Antes de ir ao laboratório

A ideia é a do Lab 2: preparar e testar tudo em simulação, e ir ao laboratório
só para gravar.

1. **Gravador** (`src/gravar.py`): lê a porta série, grava CSV com o tempo do
   Arduino, mostra a taxa de amostragem e avisa se houver linhas partidas.
2. **Simulador de IMU** (`src/simular.py`): gera dados de acelerómetro e
   giroscópio para um movimento conhecido, com desvio, escala e ruído à
   escolha. Serve para testar cada passo sabendo a resposta certa.
3. **Análise**, um script por tarefa: calibração (A, B, C), inclinómetro,
   integração do giroscópio, CI, gestos. Testados no simulador antes de verem
   dados reais.
4. **Sketch** com 115200 baud e as gamas escolhidas, pronto a carregar.

## Riscos conhecidos

- **Taxa de amostragem baixa**: se nem a 115200 passar de ~50 Hz, o soco tem
  de ser mesmo lento, e o relatório diz porquê.
- **Posição por dupla integração deriva em segundos.** A trajetória da tarefa
  6 só é credível em gestos curtos, com paragem no início e no fim.
- **Guinada**: o acelerómetro não a vê. A fusão só corrige rolamento e
  arfagem; a guinada fica só com o giroscópio, e deriva.
- **Vídeos**: o enunciado exige-os na Tarefa 2. Filmar todos os ensaios.

## Calendário proposto

| Semana | O quê |
|---|---|
| 16–22 nov | gravador, simulador e análise das tarefas 2 a 4 em dados simulados; imprimir o suporte e a placa da mão; pedir autorização para tirar um dedo da pinça |
| 23–29 nov | 1.ª ida ao laboratório: ensaios 0, A, B, C, D; rever a análise com dados reais |
| 30 nov–6 dez | CI (tarefa 5) e gestos em simulação; 2.ª ida: D2, E, F1, F2, G, e os ensaios com a IMU na pinça |
| 7–13 dez | relatório; 3.ª ida só se faltar algum ensaio |
| 14–18 dez | revisão e entrega |
