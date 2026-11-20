# Roteiro: ensaios à mão e no robô com a IMU

Lista do que é preciso fazer numa ida ao laboratório, por ordem, com a razão
de cada passo. Há um só kit, por isso a ordem segue o kit: primeiro os
ensaios à mão, depois a montagem na pinça e os ensaios no robô.

Todos os comandos correm a partir de `Lab3/`, com a `.venv` criada por
`bash setup.sh`. `gravar.py` para com Ctrl+C.

## Levar

- Portátil com a `.venv`, adaptador USB-C → USB-A.
- Kit: Uno, placa da IMU, cabo Grove de 20 cm, cabo USB comprido (da pinça
  ao portátil, com folga).
- Peça impressa (`modelos/suporte_imu/`).
- Parafusos: os 2 M2,5 × 4 do dedo da pinça, os 4 Phillips das torres do Uno,
  3 M3 × 10 com porca e anilha (IMU).
- Chave Phillips e Allen, abraçadeiras, paquímetro.
- Caixa rígida, régua de 30 cm ou mais, transferidor ou inclinómetro do
  telemóvel, fita-cola.
- Telemóvel para filmar.

## 1. Arranque

- [ ] **Carregar o sketch e fazer o teste rápido**

  ```
  arduino-cli board list
  arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:uno src/arduino/imu_bruto
  .venv/bin/python src/gravar.py 0_taxa --dur 30
  ```

  *Porquê:* o sketch lê a IMU 200 vezes por segundo e envia os valores em
  bruto, sem os arredondamentos da biblioteca da cadeira. O teste confirma o
  chip certo (`whoami=0x11`), a taxa (cerca de 200 Hz), que não se perdem
  amostras (0 falhas) e que o acelerómetro mede a gravidade (|a| perto de
  1 g). Se algum falhar, tudo o resto sai estragado. Se a porta não abrir a
  500000 baud: `BAUD` 115200 e `RATE_HZ` 100 no sketch, e `--baud 115200`.

- [ ] **A_parado: 25 min parado, com o kit frio**

  ```
  .venv/bin/python src/gravar.py A_parado --dur 1500
  ```

  Kit desligado há pelo menos 30 min, numa mesa onde ninguém mexe (não a do
  robô).

  *Porquê:* parada, tudo o que a IMU mede é erro. Dá o desvio do giroscópio
  (o valor que dá parado e que se subtrai), o ruído de cada sensor pela
  variância de Allan (que precisa de tempos longos, daí os 25 min) e a
  variação do desvio com a temperatura: a placa aquece sozinha depois de
  ligada, por isso tem de começar fria. Tarefa 2.

- [ ] **Enquanto o A grava: robô e preparação**

  ```
  .venv/bin/python ../Lab2/src/robot_info.py
  .venv/bin/python src/robo/verificar.py deriva
  .venv/bin/python src/robo/correr.py deriva --go --sem-imu --lento 2
  ```

  Preparar a caixa, a régua, a marca de fita do ensaio E e os parafusos.

  *Porquê:* `robot_info.py` confirma que o firmware deixa ler os encoders
  (versão 1.9.110 ou mais) e se há desvio de ferramenta configurado, que
  mudaria a posição da IMU nas contas. O `deriva` sem IMU e a meia velocidade
  é a primeira vez que usamos o envio de juntas ponto a ponto
  (`set_servo_angle_j`), que o Lab 2 não usou: testa-se com a pinça normal,
  onde um erro não custa nada.

## 2. Ensaios à mão

Um ficheiro e um vídeo por ensaio (o enunciado exige vídeo na Tarefa 2).
Começar e acabar com 5 s parado. Apontar no papel a ordem do que se fez.
Comando: `.venv/bin/python src/gravar.py <nome>`.

*Porquê os 5 s parados:* a análise encontra assim o início e o fim de cada
ensaio e mede o desvio do giroscópio em cada um.

- [ ] **B_seis_posicoes**: kit preso à caixa, cada face para baixo 20 s
  (+z, −z, +x, −x, +y, −y).

  *Porquê:* parado, o acelerómetro só mede a gravidade, 1 g. Comparar +1 g
  com −1 g em cada eixo dá o desvio e a escala do acelerómetro. É a
  calibração clássica da Tarefa 2.

- [ ] **B2_acaso**: 18 posições bem espalhadas, 10 s paradas cada, sem
  alinhar nada.

  *Porquê:* o ajuste a um elipsoide dá os mesmos parâmetros e ainda os eixos
  não perpendiculares, sem precisar de alinhamentos. Serve para comparar com
  o B.

- [ ] **C_rotacao_1, 2 e 3**: caixa encostada à régua. Em torno de x: +90°,
  −90°, +360°, −360°, com 5 s parados entre cada; depois y e z.

  *Porquê:* integrar o giroscópio tem de dar o ângulo conhecido; a diferença
  dá a escala do giroscópio, a parte da Tarefa 2 que o A e o B não dão.

- [ ] **D_inclinacoes_1, 2 e 3**: rolamento 0, 15, …, 90° com 5 s em cada,
  voltar a 0; depois arfagem. Ângulos ditos em voz alta para o vídeo.

  *Porquê:* são os dados das Tarefas 3 e 4. O inclinómetro (só o
  acelerómetro) e o giroscópio integrado comparam-se com os ângulos ditos.

- [ ] **D2_abanar**: como o D, com abanões de lado durante 5 s a 30° e a 60°.

  *Porquê:* com aceleração, o inclinómetro confunde-a com a gravidade e
  engana-se. É o defeito que a fusão da Tarefa 5 corrige.

- [ ] **E_mexer_parar_1, 2 e 3**: mexer à vontade 20 s, pousar exatamente na
  marca de partida, 2 min parado.

  *Porquê:* a orientação final é igual à inicial, por isso a diferença mede
  quanto o giroscópio derivou. Tarefa 4 e teste da fusão.

- [ ] **F1_acenar_1 a 5**: kit preso à mão com fita, cotovelo apoiado,
  antebraço a oscilar devagar.
- [ ] **F2_soco_1 a 5**: avançar a mão devagar ao longo da régua e voltar,
  parado 3 s no início e no fim.

  *Porquê:* são os gestos da Tarefa 6. O F1 é sobretudo rotação
  (orientação); o F2 é sobretudo translação com paragem nas pontas (para
  tentar a posição). As repetições mostram se o resultado se repete.

- [ ] **Confirmar os dados antes de desmontar**

  ```
  .venv/bin/python src/t2_calibracao.py --seis data/<B>.csv --acaso data/<B2>.csv \
      --parado data/<A>.csv --rotacao data/<C_1>.csv
  .venv/bin/python src/t3_t4_orientacao.py data/<D_1>.csv
  ```

  *Porquê:* tem de encontrar 6 troços parados no B e cerca de 18 no B2, e
  |a| perto de 1 g depois de calibrar; o segundo mostra o ângulo de cada
  paragem do D, para comparar com o vídeo. Se algo falhou, repete-se agora e
  não noutro dia.

## 3. Montagem na pinça

- [ ] **IMU na peça**: chip para fora, 3 M3 × 10 com anilha e porca por
  dentro, com a placa **encostada ao parafuso do lado do chip** antes de
  apertar.

  *Porquê:* a posição do chip vem do desenho. Encostar a placa garante que
  fica onde o modelo do robô (`src/robo/geometria.py`) a põe.

- [ ] **Uno nas torres** (parafusos Phillips pelo lado de dentro da parede) e
  **cabo Grove** ligado.

  *Porquê:* o kit vai todo na pinça, sem o acrílico.

- [ ] **Tirar um dedo da pinça** (2 M2,5 por baixo) e **aparafusar a peça**
  no lugar dele, com os mesmos parafusos. Não mexer na pinça pelo software:
  fica aberta, como está quando parada.

  *Porquê:* é a forma de prender a IMU ao robô sem mexer no punho. O modelo
  do robô conta com a pinça aberta.

- [ ] **USB preso com abraçadeiras, com folga.**

  *Porquê:* a junta 6 roda até ±270°. Um cabo preso puxava o kit ou
  desligava a meio de um ensaio.

- [ ] **Gravar 10 s com o kit montado** (`gravar.py montado --dur 10`).

  *Porquê:* confirma que nada se soltou antes de pôr o robô a mexer.

## 4. Ensaios no robô

Mão no botão de emergência. Comando:
`.venv/bin/python src/robo/correr.py <ensaio> --go`. Cada um grava
`data/<data>_<ensaio>_imu.csv` e `data/<data>_<ensaio>_robo.csv`. Se faltar
um plano em `data/planos/`: `.venv/bin/python src/robo/verificar.py`.

*Porquê o robô:* sabe a orientação e a posição da pinça a cada instante,
pelos encoders. É a verdade com que se comparam a calibração e as Tarefas 3
a 6, muito melhor do que ângulos ditos em voz alta.

- [ ] **`deriva --lento 2`**

  *Porquê:* repete o teste do envio de juntas, agora com a peça montada e a
  gravar a IMU e as juntas ao mesmo tempo, ainda a meia velocidade.

- [ ] **`poses`** (cerca de 5 min). **Mandar os dados para análise antes de
  continuar.**

  *Porquê:* 17 orientações paradas. Confirmam a orientação dos eixos do chip
  na pinça, que ainda é uma hipótese: se estiver errada, todas as
  comparações com o robô saem erradas, por isso vê-se antes dos outros
  ensaios. Também calibram o acelerómetro com orientações exatas.

- [ ] **`inclinacoes`**

  *Porquê:* a versão exata do D, para as Tarefas 3 e 4.

- [ ] **`abanar`**

  *Porquê:* a versão exata do D2, com abanões medidos: quanto o inclinómetro
  se engana com aceleração, e teste da fusão da Tarefa 5.

- [ ] **`vaivem`**

  *Porquê:* acelerações conhecidas sem rodar. Testa o critério do peso
  dinâmico da fusão (confiar menos no acelerómetro quando |a| se afasta de
  1 g).

- [ ] **`acenar`**

  *Porquê:* a versão exata do gesto F1, com a verdade, para a Tarefa 6.

- [ ] **`rotacoes`**, por último, a vigiar o cabo a cada volta.

  *Porquê:* a junta 6 a velocidades conhecidas (30, 90 e 140 °/s) dá a
  escala do giroscópio com mais rigor do que o C. Fica para o fim porque é a
  que mais torce o cabo.

## 5. Fecho

- [ ] **Repor o dedo original da pinça.**

  *Porquê:* o robô é partilhado com outros grupos.

- [ ] **Commit e push dos dados** (`data/*.csv`).

  *Porquê:* ficam guardados e o grupo tem acesso para a análise.

- [ ] **Vídeos na pasta partilhada do grupo**, com o mesmo nome do ficheiro de
  dados (não vão para o git).

  *Porquê:* cada vídeo fica ligado ao seu ficheiro, para o relatório e para
  confirmar ângulos.

## Se o tempo apertar

Cortar repetições: 1 de cada no C, no D e no E, e 3 em vez de 5 no F. O A, o
B, o B2 e os ensaios no robô não se cortam.
