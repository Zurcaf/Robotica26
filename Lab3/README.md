# Lab 3 — Estimação de trajetória com uma IMU

Enunciado: [lab_3_2026.pdf](lab_3_2026.pdf). Prazo: **18 dez 2026, 23:59:59**. Grupos até 3.
Relatório: sem limite, mas "6 páginas A4 devem chegar"; pouco texto.

- [PLANO.md](PLANO.md) — plano de ataque: o que preparar, que ensaios gravar no laboratório e porquê.
- [ROTEIRO_17NOV.md](ROTEIRO_17NOV.md) — o que fazer na 1.ª ida ao laboratório, com os comandos.
- [ROTEIRO_ROBO.md](ROTEIRO_ROBO.md) — lista do que fazer numa ida ao laboratório (ensaios à mão, montagem na pinça, ensaios no robô), com a razão de cada passo.
- [src/](src/README.md) — sketch do Arduino, gravador, simulador, análise das tarefas 2 a 4 e ensaios no robô (verificados no MuJoCo). Instalar com `bash setup.sh`.
- `resultados/robo/` — imagens dos ensaios do robô no MuJoCo, com o suporte e o kit.
- [report/ROB_LAB3_Report/](report/ROB_LAB3_Report/) — relatório (esqueleto, formato dos Labs 1 e 2); ver [report/README.md](report/README.md).
- `modelos/` — suporte da IMU para a pinça do robô, placa para a mão, e o modelo da pinça do fabricante.
- `scripts_curso/` — scripts de exemplo da página da cadeira:
  `get_imu_data.ino` (Arduino, lê a IMU Grove) e
  `lab3_pyserial_comm_arduino.py` (lê os dados por porta série no PC).
