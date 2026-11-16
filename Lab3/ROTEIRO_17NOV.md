# Segunda, 17 nov — roteiro da 1.ª ida ao laboratório

Objetivo: gravar todos os ensaios que não precisam do robô e confirmar no
robô o que falta para o suporte. Um colega pôs o suporte a imprimir na
Composites a 16 nov; a peça é verificada na terça, 17 nov.

## Levar

- Portátil com a `.venv` do Lab 3 (`bash setup.sh`, já feito no Mac do Afonso)
  e o `arduino-cli` (já instalado com o Homebrew).
- Adaptador USB-C → USB-A, para o Uno.
- Caixa rígida ou esquadro (para as seis posições e as rotações), fita-cola.
- Transferidor ou o inclinómetro do telemóvel, régua de 30 cm ou mais.
- Paquímetro, para medir o kit.
- Telemóvel e apoio para filmar (a Tarefa 2 exige vídeo de cada ensaio).
- Opcional, para o ensaio da temperatura: um saco de congelação, para
  arrefecer o kit no frigorífico do laboratório, se houver.

## 1. Carregar o sketch (5 min)

O sketch não precisa de bibliotecas. A partir de `Lab3/`:

```
arduino-cli board list
arduino-cli compile --fqbn arduino:avr:uno src/arduino/imu_bruto
arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:uno src/arduino/imu_bruto
```

A porta é a que o `board list` mostrar (num clone pode ser
`/dev/cu.usbserial-XXXX` ou `/dev/cu.wchusbserial-XXXX`). Também serve o
Arduino IDE: abrir `src/arduino/imu_bruto/imu_bruto.ino`, placa "Arduino Uno",
carregar.

Teste de 10 s (é também o ensaio 0):

```
.venv/bin/python src/gravar.py 0_taxa --dur 30
```

Tem de mostrar `whoami=0x11`, cerca de 200 Hz, 0 falhas e |a| perto de 1 g.
Se não abrir a 500000 baud, baixar `BAUD` no sketch para 115200 e `RATE_HZ`
para 100, carregar outra vez e usar `--baud 115200`.

## 2. Ensaios sem o robô

Um ficheiro e um vídeo por ensaio. O gravador para com Ctrl+C. Começar e
acabar cada ensaio com 5 s parado. Apontar em papel a ordem do que se fez.

| Ordem | Comando | O que fazer |
|---|---|---|
| A / T | `gravar.py A_parado --dur 1500` | **Primeiro de tudo**, com o kit frio (desligado há pelo menos 30 min, ou do frigorífico). Ligar e gravar logo, 25 min sem tocar na mesa. Entretanto, fazer a parte 3 (robô). |
| B | `gravar.py B_seis_posicoes` | Kit preso à caixa. Cada face para baixo 20 s: +z, −z, +x, −x, +y, −y. |
| B2 | `gravar.py B2_acaso` | 18 posições bem espalhadas, 10 s paradas cada, sem alinhar nada. |
| C | `gravar.py C_rotacao_1` | Caixa encostada à régua. Em torno de x: +90°, −90°, +360°, −360°, com 5 s parados entre cada. Depois y e z. Repetir em `C_rotacao_2` e `C_rotacao_3`. |
| D | `gravar.py D_inclinacoes_1` | Rolamento 0, 15, …, 90° com 5 s em cada, voltar a 0; depois arfagem. Ângulos do transferidor ou do telemóvel, ditos em voz alta para o vídeo. Três vezes. |
| D2 | `gravar.py D2_abanar` | Como D, mas a 30° e a 60° dar abanões de lado durante uns 5 s. |
| E | `gravar.py E_mexer_parar_1` | Mexer à vontade 20 s, pousar exatamente na posição de partida (marcar com fita na mesa), 2 min parado. Três vezes. |
| F1 | `gravar.py F1_acenar_1` | Kit na mão (a placa ainda não está impressa: fita-cola). Cotovelo apoiado, antebraço a oscilar devagar. Cinco vezes. |
| F2 | `gravar.py F2_soco_1` | Mão a avançar devagar ao longo da régua e a voltar, parada 3 s no início e no fim. Cinco vezes. |

Todos os comandos são `.venv/bin/python src/...`, a partir de `Lab3/`.

**Antes de sair**, ver se os dados prestam:

```
.venv/bin/python src/t2_calibracao.py --seis data/<B>.csv --acaso data/<B2>.csv \
    --parado data/<A>.csv --rotacao data/<C_1>.csv
.venv/bin/python src/t3_t4_orientacao.py data/<D_1>.csv
```

O primeiro tem de encontrar 6 troços parados no B e cerca de 18 no B2, e dar
|a| perto de 1 g depois de calibrar. O segundo imprime o ângulo de cada
paragem: comparar com os ângulos ditos no vídeo.

## 3. No robô, enquanto o ensaio A grava

Nada disto mexe no robô, exceto o último ponto.

1. **Os dedos estão montados com a lâmina para fora?** É o que o suporte
   assume (pinça fechada: 22 mm entre dedos). Fotografar.
2. **A pinça fica fechada quando está parada?** Com ela aberta, o suporte fica
   8 mm mais para fora.
3. **Os parafusos dos dedos são M2,5 e entram por baixo?** Ver sem tirar.
4. **Medir o kit**: tamanho do acrílico, espessura da parede às pontas das
   fichas do Arduino, e onde está o chip da IMU (distâncias às arestas da
   placa). Corrigir `KIT_*` e `P_F_IMU` em `src/robo/geometria.py`, e correr
   `verificar.py` outra vez.
5. **Versão do firmware** e desvio da ferramenta configurado:
   `.venv/bin/python ../Lab2/src/robot_info.py`.
6. **Opcional, para testar o envio de juntas** (`set_servo_angle_j`, que o
   Lab 2 não usou), com a pinça normal e sem IMU, a metade da velocidade:

   ```
   .venv/bin/python src/robo/verificar.py deriva
   .venv/bin/python src/robo/correr.py deriva --go --sem-imu --lento 2
   ```

   Mão no botão de emergência. O plano foi verificado com o suporte no lugar
   de um dedo; com o dedo normal há mais folga, não menos.

## 4. Perguntas ao professor

- O filtro de Kalman é pedido ou é extra?
- Na Tarefa 6, espera só a orientação ou também a posição?
- "O melhor parâmetro" tem de ser um valor fixo, ou pode ser um peso que se
  adapta ao movimento?
