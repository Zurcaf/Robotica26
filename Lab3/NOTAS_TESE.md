# Notas para a tese (Afonso) — o que o Lab 3 ensina sobre IMUs

Sensor da tese: **LSM6DSV320X** (ST), montado numa prótese ancorada no osso de
um cão. Objetivo: orientação da pata, impactos, e o que mais se mostrar
interessante. Estas notas não fazem parte da entrega do Lab 3.

## O que se transfere diretamente

| Na tese | No Lab 3 |
|---|---|
| Modelo de erros do sensor (desvio, escala, ruído) | tarefas 1 e 2 |
| Calibrar sem equipamento | ensaio B2 (posições ao acaso) |
| Orientação ao longo do passo | tarefas 3, 4 e 5 |
| Não confiar no acelerómetro durante o impacto | ensaio D2 |
| Deriva em registos longos | ensaios E, G e T |
| Orientação do chip em relação à prótese | o que o robô dá no Lab 3 |
| Corrigir quando a pata está no chão | "velocidade zero quando pára" (ensaio G) e Kalman com o desvio no estado |

## Hábitos a manter

1. Gravar sempre os dados em bruto, com a temperatura do chip.
2. Começar e acabar cada registo com 5 a 10 s parado.
3. Anotar que sensor é e que calibração se lhe aplica.

## Temperatura (valores de ficha, por confirmar)

| | Giroscópio | Acelerómetro |
|---|---|---|
| LSM6DSV320X | ±0,006 °/s por °C (valor do irmão LSM6DSV32X) | ±0,07 mg por °C (canal de baixa aceleração) |
| ICM-20600 (lab) | ±0,02 °/s por °C (fonte indireta) | ±0,5 a 1 mg por °C (fonte indireta) |

Com 15 °C de variação, o giroscópio do LSM6DSV320X ganha ~0,09 °/s de desvio:
5° por minuto se não for corrigido. No acelerómetro é desprezável (0,06°).
Método para caracterizar: ensaio T do plano, repetido no chip da tese.

## Específico da tese (o Lab 3 não cobre)

- **Dois acelerómetros no chip** (a confirmar na ficha): baixa aceleração (até
  ±16 g) e alta (até ±320 g). O de alta não se calibra com a gravidade (1 g
  em 320 g perde-se no ruído); calibra-se contra o de baixa, em movimentos
  que os dois apanhem.
- **Impactos**: curtos (milissegundos) e, com a prótese no osso, sem
  amortecimento. Precisam do canal de alta aceleração e de vários kHz de
  amostragem.
- **Montagem rígida**: sem movimento de pele ou pelo. A orientação do chip em
  relação à prótese sai do desenho e confirma-se uma vez na bancada; a
  calibração de bancada vale no uso real.
- O chip traz fusão de orientação interna: usar como comparação, não como
  resultado.

## Grandezas que podem interessar

Pico de impacto por passo; ângulo da prótese no contacto com o chão; duração
do apoio e do balanço; regularidade entre passos; soma dos impactos ao longo
do dia, como carga acumulada no implante.

## Kalman e CI, em duas linhas

O giroscópio faz a previsão, o acelerómetro a correção. O Kalman assume que os
erros de amostras diferentes são independentes; quando não são (desvio mal
calibrado, aceleração que dura), fica confiante de mais. A CI não assume
isso. Remédios: desvio no estado do filtro; mais incerteza no acelerómetro
quando o módulo da aceleração se afasta de 1 g.

Fichas: <https://www.st.com/resource/en/datasheet/lsm6dsv320x.pdf>,
<https://invensense.tdk.com/download-pdf/icm-20600-datasheet>

---

# Perguntas e respostas (13/11/2026)

## Plano de validação

O orientador sugeriu uma sala de captura de movimento com o cão, e usar o robô.
Os dois completam-se:

- **Robô (bancada):** a prótese na pinça, em movimentos conhecidos. Valida o
  algoritmo e mede duas coisas que hoje não se sabem (ver "posição do sensor"):
  a orientação do chip dentro da prótese e a distância ao eixo.
- **Captura de movimento (cão):** valida no uso real. Precisa de pelo menos 3
  marcadores presos rigidamente à prótese, para dar orientação e não só
  posição, e de um acontecimento comum para acertar os relógios (uma pancada
  seca que apareça nos dois registos).
- **Limite:** a captura de movimento amostra a centenas de Hz. Serve para a
  orientação, não para validar o pico dos impactos; para isso é preciso uma
  plataforma de forças ou outro acelerómetro de referência.

## Suavizador (o trabalho é todo em pós-processamento)

Um filtro normal, em cada instante, só conhece o passado. Se o resultado não é
preciso em tempo real, pode usar-se também o futuro.

Exemplo: a pata sai do chão, anda no ar meio segundo e volta a assentar. No
chão a orientação conhece-se bem (pata parada, gravidade). No ar só há o
giroscópio, e o erro cresce.

- **Filtro:** o erro cresce durante o voo e é corrigido de repente ao
  assentar. A meio do voo já vai torto, e há um salto no fim.
- **Suavizador:** depois de chegar ao fim, volta para trás e distribui a
  correção por todo o voo. O erro é máximo a meio (e não no fim), fica em
  menos de metade, e não há saltos.

Na prática corre-se o Kalman para a frente, guardando os resultados, e depois
uma passagem para trás. O nome a procurar é "RTS smoother"
(Rauch–Tung–Striebel). Não muda os dados a gravar, só o processamento.

## Giroscópio e impactos

Na ficha técnica chama-se "linear acceleration sensitivity" ou
"g-sensitivity" do giroscópio, em °/s por g. Não encontrei esse valor para o
LSM6DSV320X; muitas fichas de sensores deste tipo não o dão. Procurar na
tabela de características do giroscópio e nos "absolute maximum ratings"
(choque máximo suportado).

Ordem de grandeza habitual nestes sensores: centésimas de °/s por g (valor
típico da classe, não deste chip). Com 0,01 °/s por g, um impacto de 100 g
durante 5 ms dá 1 °/s durante 5 ms: 0,005° de erro. O efeito direto é
desprezável. O que pode importar:

1. **Ressonância:** um choque seco pode pôr a estrutura interna do giroscópio
   a vibrar e dar leituras falsas durante alguns milissegundos.
2. **Saturação:** se a rotação real no impacto passar da gama escolhida
   (o chip vai até ±4000 °/s).
3. **Desvio alterado depois do choque.**

**Teste de bancada (barato, fazer cedo):** prótese pousada, sem rodar; dar
pancadas de intensidade crescente; ver o que o giroscópio diz durante e
depois. Devia dizer zero. Tudo o que aparecer é este efeito. Repetir com a
gama e os filtros que se vão usar.

## Ângulos, quaterniões e o bloqueio a 90°

Descrever uma orientação com três ângulos (rolamento, arfagem, guinada) é
dizer "roda primeiro em torno deste eixo, depois daquele, depois do outro".
Quando o ângulo do meio chega a 90°, o primeiro e o terceiro eixos ficam
alinhados e passam a fazer a mesma coisa: perde-se um grau de liberdade na
descrição. A orientação continua a existir, mas há infinitas combinações de
ângulos que a dão, e a conta que transforma a velocidade do giroscópio em
variação dos ângulos tem uma divisão por zero.

É o mesmo fenómeno da singularidade do pulso do robô no Lab 2: com a junta 5
a zero, as juntas 4 e 6 alinhavam-se e as velocidades disparavam.

Um quaternião guarda a orientação de outra forma: "uma rotação de ângulo θ em
torno do eixo n", em quatro números:

    q = ( cos(θ/2),  n·sin(θ/2) )

Não tem nenhuma posição especial. Integra-se o giroscópio diretamente no
quaternião, normaliza-se (os quatro números têm de ter tamanho 1) e só no fim,
para mostrar gráficos, se converte para ângulos. Se a pata passar pelos 90°, o
gráfico dos ângulos pode dar um salto, mas a orientação por dentro está certa.

## Porque é que a vertical é especial

Com o giroscópio sozinho, as três rotações derivam igualmente. A diferença
está em quem as corrige:

- **Inclinação (rolamento e arfagem):** a gravidade é uma referência absoluta.
  Com o sensor parado, o acelerómetro diz para que lado é "para baixo", e isso
  corrige as duas.
- **Rotação em torno da vertical (guinada):** rodar em torno da vertical não
  muda a direção da gravidade vista pelo sensor. O acelerómetro não a vê, logo
  ninguém a corrige.

O magnetómetro dá a referência que falta (o norte), mas é perturbado por
metal e correntes por perto, e também precisa de calibração. Não é
indispensável: se interessa a inclinação e o movimento dentro de cada passo, a
deriva em meio segundo de voo é mínima (0,1 °/s × 0,5 s = 0,05°), e cada apoio
no chão volta a acertar a inclinação. Só faz falta se a tese precisar da
direção para onde o cão vai ao longo de muito tempo.

## Posição do sensor na prótese ("ficou onde cabia")

Dois factos separam o que preocupa do que não preocupa:

- **O giroscópio mede o mesmo em qualquer ponto de um corpo rígido.** A
  orientação por integração do giroscópio não depende de onde o chip está nem
  de a que distância fica da articulação.
- **O acelerómetro depende do sítio.** Num ponto a uma distância r do eixo de
  rotação, mede a mais ω²·r (para o centro) e α·r (tangencial). Com a pata a
  rodar a 573 °/s (10 rad/s) e r = 5 cm, são 5 m/s², cerca de 0,5 g. Não é
  pequeno.

Consequências:

- **Orientação:** não é afetada. O acelerómetro só é usado para corrigir a
  inclinação quando a pata está parada no chão, e aí ω é zero.
- **Impactos:** o valor medido é a aceleração **no sítio do chip**. É válido e
  comparável entre passos e entre dias. Só é preciso r para o converter na
  aceleração noutro ponto (por exemplo, na ligação ao osso).
- **Orientação do chip desconhecida:** não é problema, mede-se. Com a prótese
  na pinça do robô:
  1. parada em várias orientações conhecidas, a gravidade dá a rotação entre
     os eixos do chip e os da prótese;
  2. a rodar a velocidade constante conhecida em torno de um eixo conhecido,
     a aceleração centrípeta dá a distância: r = a / ω². A 180 °/s e r = 5 cm
     são 50 mg, que se medem bem.

Se existir o desenho 3D da prótese com a placa lá dentro, tira-se dali um
primeiro valor e o robô confirma.

## Outras perguntas a ter presentes

- **Gama e taxa de amostragem:** se o impacto passar da gama, o pico corta-se;
  se a taxa for baixa, o pico cai entre amostras; os filtros internos do chip,
  mal escolhidos, alisam o que se quer medir. Decide-se antes de gravar.
- **Ruído e limite do sensor:** variância de Allan, sobre um registo longo com
  o sensor parado. Diz até que duração vale a pena integrar.

Para levar ao orientador primeiro: a validação e o suavizador. Para testar
cedo na bancada: giroscópio sob impacto, e gama e taxa de amostragem.
