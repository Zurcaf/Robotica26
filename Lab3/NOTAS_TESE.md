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
