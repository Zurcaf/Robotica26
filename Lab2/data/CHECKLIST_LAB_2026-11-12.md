# Laboratório 12/11/2026 — guia para fazer sozinho

Tudo o que precisas está aqui. Lê de cima para baixo; cada passo diz o que
escrever, o que deves ver e o que fazer se correr mal. Os passos 1–3 não mexem
o robô. **Se só fizeres o passo 3 já valeu a pena.**

Antes de começar: botão de emergência à mão, meio metro livre à volta do robô,
nada em cima da mesa na zona do braço.

---

## 0. Ligação ao robô

Precisas de um cabo Ethernet entre o PC e o controlador (caixa do robô). Se o
teu portátil só tem USB-C: pedir um adaptador no lab, ou usar um PC do lab
com porta de rede.

**IP fixo no PC** (o robô é `192.168.1.158`):

- Windows: Definições → Rede → Ethernet → Atribuição de IP → Editar → Manual →
  IPv4 ligado → IP `192.168.1.50`, máscara `255.255.255.0` (ou prefixo `24`),
  gateway em branco → Guardar.
- Mac: Definições → Rede → Ethernet/adaptador → Detalhes → TCP/IP →
  Configurar IPv4: Manualmente → IP `192.168.1.50`, máscara `255.255.255.0`.

**Teste** (terminal / PowerShell):

```
ping 192.168.1.158
```

Tem de responder. Se não responder: cabo mal encaixado (a 6/11 houve várias
quedas "Media disconnected"; tirar e voltar a pôr resolveu), ou o IP fixo não
ficou gravado. Também podes abrir no browser `http://192.168.1.158:18333`
(UFactory Studio): se abrir, a rede está boa.

## 1. Código no PC

Se for o teu Mac, já tens tudo em `~/Developer/Robotica26/Lab2`; salta para o 2.

Num PC do lab:

```
git clone https://github.com/Zurcaf/Robotica26.git
cd Robotica26/Lab2
pip install numpy matplotlib xarm-python-sdk
```

Se `pip` não existir no Windows, usa `py -3 -m pip install ...` e depois
`py -3 src/...` em vez de `python src/...`. Não é preciso instalar `mujoco`.

Teste sem robô (tem de acabar em `VIÁVEL`):

```
python src/trajectory.py
```

## 2. Teste de ligação (não mexe o robô)

```
python src/ik_compare.py --n 10
```

Se imprimir "10 pontos gravados" a ligação ao controlador funciona. (Escreve
por cima de `data/ik_compare.csv` — não faz mal, está no git.)

Se der erro de ligação: ver o passo 0. Se disser que o robô está em erro:
carrega no botão de emergência, roda-o para soltar, e corre outra vez (o script
limpa os erros ao ligar).

## 3. Configuração do controlador (não mexe o robô) — O MAIS IMPORTANTE

```
python src/robot_info.py
```

Imprime e grava `data/robot_info.txt`. Interessa:

- `tcp_load` — se for `[0, ...]`, a pinça (0.29 kg) não está declarada: é a
  hipótese do erro C31 de 5/11;
- parâmetros D-H do controlador (`get_dh_params`) — fecha a Tarefa 4;
- `tcp_speed_limit`, `tcp_acc_limit`, `joint_speed_limit`.

**Não mudes nada no controlador** (o robô é partilhado).

## 4. Robô ao zero

```
python src/ir_para_zero.py
```

Mostra o plano e pede confirmação (`sim`). Vai por etapas a 15 °/s. Não uses o
`move_gohome` / botão "home" do Studio: dá C22 (autocolisão) a partir de
algumas poses.

## 5. Swing V3 com a ponta mais alta (o que falta para entrar no relatório)

Primeiro o ensaio, que não mexe:

```
python src/run_robot_v3.py --bola-demo --dj2 -6
```

Deve dizer `swing V3 VIÁVEL`, `duração 20.4 s`, `bola em [377.1 23.1 61.]` e
`ENSAIO — nada foi enviado`. Depois a sério:

```
python src/run_robot_v3.py --go --lento 2 --bola-demo --dj2 -6
```

O que acontece: aproximação por etapas a 20 °/s a partir do zero (estica o
braço, roda o pulso longe da base, inclina, endereço) → pede **Enter** → swing
de 20 s, a base roda 125°, a ponta passa a 61 mm da mesa → grava
`data/run_v3_dj2-6_lento2.csv` (nunca escreve por cima; se já existir
acrescenta a hora).

Se der tempo, repetir uma vez (para a repetibilidade) e, só se o primeiro
correu limpo, a velocidade planeada:

```
python src/run_robot_v3.py --go --lento 1 --bola-demo --dj2 -6
```

No fim volta ao zero com o `ir_para_zero.py`.

## 6. Opcional: GOLFISTA (o swing do robô direito, nunca correu no robô)

```
python src/run_robot.py --golfista --lento 2                        # ensaio, não mexe
python src/run_robot.py --go --metodo servo --lento 2 --golfista    # grava data/run_servo_lento2_golfista.csv
```

Aproxima-se junta a junta a 20 °/s, pede `sim`, faz o swing em 21 s. O braço
vai quase esticado à frente do robô, ponta a 390 mm da base. Se correr bem,
`--lento 1` (o verificador avisa que o flange pede 258 mm/s; é esperado).

## 7. Guardar os dados (não saias sem isto)

```
git add data/
git commit -m "Lab2: dados de 12/11 (robot_info, V3 dj2 -6, golfista)"
git push
```

Se o `git push` pedir login num PC do lab e não conseguires, **envia os
ficheiros para ti** (email / WhatsApp / pen): `data/robot_info.txt` e todos os
`data/run_*.csv` novos. Depois comitamos em casa.

---

## Se algo correr mal

| O que vês | O que é | O que fazer |
|---|---|---|
| `C31` | deteção de colisão (corrente anormal) | o script pára e grava o que tem; ver se algo tocou; correr outra vez (limpa o erro) |
| `C22` | autocolisão prevista pelo controlador | não forçar; `ir_para_zero.py` e repetir |
| `C24` | velocidade acima do limite | só acontece com `pontos --lento 1`; usa `servo` ou `--lento 2` |
| `Media disconnected` / timeout | cabo de rede | tirar e pôr o cabo; `ping` |
| robô não mexe, sem erro | emergência carregada ou motores desligados | soltar a emergência; o script faz `motion_enable` ao ligar; se não, Studio → Enable |
| script pára a meio | grava sempre o CSV com o que recolheu | não apagar o CSV; anotar em que ponto parou |

Anota num ficheiro `data/LAB_2026-11-12.md` (ou no telemóvel): hora, comando,
correu / parou onde, algo estranho. Três linhas chegam.
