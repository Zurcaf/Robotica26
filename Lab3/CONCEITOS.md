# Conceitos para o relatório

Explicações curtas dos conceitos usados nas Tarefas 2 a 5, com os nomes em
português e em inglês (os artigos usam os nomes em inglês) e os números dos
nossos dados. Servem para escrever o relatório com os termos certos.

## O desvio (bias) do giroscópio

Parado, o giroscópio devia ler 0 °/s, mas lê um valor fixo: o **desvio**
(*bias*). No nosso, cerca de (−0,3; −0,6; −0,3) °/s (teste na mesa e
calibração com o robô, 20/11). Integrado, um desvio de 0,6 °/s dá 60° de erro
em 100 s, por isso tirá-lo é o passo mais importante da calibração (Tarefa 4:
sem calibrar, 60° de erro no fim das inclinações; calibrado, 2,7°).

O desvio não é perfeitamente fixo. Muda por duas razões diferentes, que é
preciso separar:

| | Ruído e passeio aleatório | Temperatura |
|---|---|---|
| O que é | aleatório, sem direção | sistemático, previsível |
| Mede-se com | variância de Allan | reta desvio = f(temperatura) |
| Corrige-se? | não; só se escolhe o melhor tempo de média | sim, com a reta |

## Variância de Allan (*Allan variance*)

**A pergunta a que responde:** quanto tempo se deve fazer a média das leituras
paradas para conhecer melhor o desvio?

**Como se calcula**, sobre um registo parado (ensaio A):

1. escolhe-se um tempo de média τ (por exemplo 1 s);
2. corta-se o registo em pedaços de τ e faz-se a média de cada pedaço;
3. vê-se quanto difere a média de cada pedaço da do pedaço seguinte
   (metade da média das diferenças ao quadrado);
4. repete-se para muitos τ (de 0,01 s a centenas de segundos) e desenha-se o
   **desvio de Allan** σ(τ) (*Allan deviation*, a raiz) contra τ, com os dois
   eixos em escala logarítmica.

**O que mostra a curva:**

- **Para τ pequenos, desce** com inclinação −½: é o **ruído branco** (*white
  noise*). Cada leitura tem um pouco de ruído (na mesa, ±0,05 °/s de amostra
  para amostra), que se apaga ao fazer a média. O valor a τ = 1 s chama-se
  **densidade de ruído** N. No giroscópio, integrado, este ruído dá um erro de
  ângulo que cresce com √t: o **passeio aleatório do ângulo** (*angle random
  walk*, ARW). No acelerómetro, o equivalente é o *velocity random walk*.
- **Chega a um mínimo**: a **instabilidade do desvio** (*bias instability*). É
  o melhor que se consegue conhecer o desvio a fazer médias, e o τ desse
  mínimo é o tempo de média ideal.
- **Para τ grandes, volta a subir** com inclinação +½: o **passeio aleatório
  do desvio** (*rate random walk*, parâmetro K). O desvio vai mudando devagar e
  ao acaso. Por exemplo, num eixo: −0,600 °/s no minuto 0, −0,607 no minuto
  5, −0,598 no minuto 10. Com pedaços de 10 min, a média dos minutos 0–10 está
  certa *para esses minutos*, mas a dos minutos 10–20 já é outra: duas médias
  seguidas voltam a diferir. "Piorar" não quer dizer que a média longa esteja
  errada; quer dizer que **um desvio medido há 10 minutos já não serve
  exatamente para agora**.

**Consequência prática:** o desvio mede-se perto de quando se usa. Por isso
cada ensaio do robô tem 3 s parados no início e no fim, e não se usa um valor
calibrado de manhã para um ensaio da tarde.

## Deriva com a temperatura

Quando o chip aquece, o desvio desloca-se sempre no mesmo sentido (por
exemplo, de −0,50 °/s a 22 °C para −0,60 °/s a 27 °C). Não é ruído: mede-se
uma vez e corrige-se sempre, com uma reta do desvio em função da temperatura,
que a IMU dá em cada amostra. A 20/11 a IMU foi de 25,8 a 27,0 °C ao longo da
manhã.

**Porque se mistura com Allan:** se a placa aquecer durante o ensaio A, o
desvio anda e a curva de Allan sobe nos τ grandes como se o sensor fosse pior
(uma subida com inclinação +1, a *rate ramp*). Por isso o ensaio A:

- começa com o kit **frio**, para a temperatura variar bastante e a reta
  sair bem medida;
- tira-se primeiro o efeito da temperatura, e só depois se faz a curva de
  Allan com o que sobra, que é só a parte aleatória.

No simulador, 10 min a aquecer 6 °C não chegavam: o passeio aleatório do
desvio confundia-se com a temperatura e a reta saía errada. Daí os 25 min.

## Para que servem estes números

- **Tarefa 2:** caracterizam o ruído de cada sensor.
- **Tarefa 5 (fusão):** o N do acelerómetro diz quanto confiar nele (o R do
  filtro de Kalman); o N e o K do giroscópio dizem quanto erro ele acumula por
  segundo (o Q).

## Ruído do sensor e vibração do robô

O ruído do sensor mede-se **na mesa**. No robô parado há uma vibração dos
servos por cima dele (os motores estão sempre a corrigir a posição):

| Kit parado | ruído acc [mg] | ruído giro [°/s] |
|---|---|---|
| mesa | 1,6 / 1,8 / 3,1 | 0,05 / 0,05 / 0,04 |
| mesa do robô, com e sem o DC/DC | 1,6 / 1,4 / 3,0 | 0,05 / 0,05 / 0,04 |
| no braço, motores a segurar | 14 / 6,5 / 13,7 | 0,42 / 0,93 / 0,27 |

A mesa do robô não vibra; a vibração vem do braço. Os troços parados do robô
servem para médias (a vibração anda para um lado e para o outro), mas não
para medir o ruído nem para a variância de Allan. Na fusão, o R ou o peso
ajustados com o ruído da mesa confiam demais no acelerómetro quando ele está
no robô.

## Inclinómetro, giroscópio e fusão (Tarefas 3 a 5)

**Erro de inclinação:** em cada instante, cada método diz para onde é a
vertical vista pela IMU; o robô diz qual é a verdadeira; o erro é o ângulo
entre as duas.

- **Inclinómetro (acelerómetro, Tarefa 3):** parado, o acelerómetro só sente
  a gravidade e aponta para baixo. Quando acelera, sente também a aceleração e
  engana-se. Não acumula erro: em cada instante parte do zero. Nas
  inclinações do robô: erro médio 1,8°, mas até 13° nos movimentos.
- **Giroscópio integrado (Tarefa 4):** soma a velocidade de rotação ao longo
  do tempo. A aceleração não o afeta, mas um erro pequeno na velocidade
  vai-se somando: deriva. Nas inclinações: 2,7° ao fim de 110 s, calibrado.
- **Fusão (Tarefa 5):** usa o giroscópio a curto prazo (bom nos movimentos)
  e o acelerómetro a longo prazo (não deriva), para ficar sem os saltos do
  primeiro e sem a deriva do segundo.

## Calibração com o robô (20/11)

Calibrada com as poses e as rotações da 1.ª sessão (`src/t2_robo.py`),
verificada na 2.ª sessão de poses, que não entrou no ajuste:

| | Acelerómetro | Giroscópio |
|---|---|---|
| Desvio | (−20,9; 4,0; 1,4) mg | (−0,31; −0,62; −0,31) °/s |
| Escalas | 1,006 / 1,001 / 0,989 | 0,996 / 0,994 / 0,991 |
| Rotação da placa na peça | 4,6° | 4,3° |

A rotação da placa sai igual pelos dois sensores, de forma independente.
Na verificação, o ângulo do acelerómetro à gravidade passa de 4,6° para 0,8°
e o erro do giroscópio em movimento de 1,5 °/s para 0,23 °/s.

**Cuidado com a escala do giroscópio:** comparar o giroscópio com uma verdade
ruidosa (a velocidade angular tirada das juntas por diferenças finitas) puxa a
escala estimada para baixo (dava 0,965, falso). Alisando os dois sinais por
igual, sai 0,991 a 0,996. É o efeito de ter ruído na variável de entrada de
uma regressão (*regression dilution*).
