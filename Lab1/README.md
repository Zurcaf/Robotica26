# Lab 1 — Swing de golfe em MuJoCo

Enunciado: [lab_1_2026.pdf](lab_1_2026.pdf). Prazo: 9 out 2026.

## Setup (Mac / Windows / Linux)

```bash
cd Lab1
python -m venv .venv
# Mac/Linux:
source .venv/bin/activate
# Windows:
.venv\Scripts\activate

pip install -r requirements.txt
```

## Correr

Todos os comandos a partir da pasta `Lab1/`:

```bash
python src/run_pendulum.py            # mede o período do pêndulo (valida a física)
python src/run_pendulum.py --view     # viewer interativo
python src/run_pendulum_ball.py       # pêndulo a bater na bola
python src/run_pendulum_ball.py --view
```

Também podes abrir qualquer XML diretamente no viewer:

```bash
python -m mujoco.viewer --mjcf=models/00_pendulum.xml
```

## Estrutura

```
models/   ficheiros XML do MuJoCo (e STL, se houver)
src/      scripts Python
report/   relatório, sketches, gráficos
```

## Versões

| Tag   | Conteúdo |
|-------|----------|
| v13   | pêndulos + golfista 2 dof + 1 teste + relatório |
| v17   | + pulso (3 dof), parâmetros reais, bateria de testes com ruído por dof |
| v20   | + STL, braço duplo, análise de sensibilidade |
