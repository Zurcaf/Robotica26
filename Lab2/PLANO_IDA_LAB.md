# Próxima ida ao laboratório (antes da entrega de 13/11)

Tudo a partir de `Lab2/`, com a `.venv` ativa e o portátil ligado ao robô por
cabo (`192.168.1.158`). Por ordem de prioridade:

## 1. Ler a configuração do controlador (não mexe o robô, ~1 min)

```bash
python src/robot_info.py          # grava data/robot_info.txt
```

Fecha três pontos do relatório:
- `tcp_load`: se a massa da pinça (0,29 kg) não estiver configurada, é a
  explicação mais provável do C31 (Secção 3.3).
- `tcp_acc_limit`, `joint_acc_limit`, `tcp_jerk`: limites que podem explicar o
  atraso à velocidade planeada (Secção 3.2). A trajetória pede no máximo
  227 °/s² nas juntas (limite do fabricante 1145) e ~430 mm/s² no flange.
- `DH do controlador`: compara com a Tabela 2 e fecha a diferença de 1,4 mm
  (Secção 4.3).

## 2. Swing V3 com 30 mm de folga à mesa (só se o V3 ficar no relatório)

A bola tem de estar à altura nova (a ponta passa a 61 mm da mesa em vez de 45).

```bash
python src/run_robot_v3.py --lento 2 --bola-demo --dj2 -6        # ensaio a seco (sem --go)
python src/run_robot_v3.py --go --lento 2 --bola-demo --dj2 -6
python src/plot_run.py --v3                                       # com o CSV novo
```

Depois, no `main.tex`, trocar `\segundoswingfalse` por `\segundoswingtrue`.
Se não houver tempo, o V3 fica de fora (já está desligado).

## 3. Anotar e fotografar

- O que serviu de bola e a que altura estava; como ficou a mesa (Secção 2.5).
- Uma foto do robô no endereço, com a bola (útil para o relatório).

## 4. Opcional

```bash
python src/run_robot.py --go --metodo servo --lento 2 --golfista
```
