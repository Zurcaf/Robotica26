"""Vídeo do ponto de situação do Lab 3: a peça, os ensaios do robô no MuJoCo e
os resultados sobre dados simulados.

    .venv/bin/python video/video_estado.py [SAIDA.mp4]      (a partir de Lab3/)

Por omissão grava video/media/estado_lab3.mp4 (fora do git). Precisa de
openscad e ffmpeg, e dos planos em data/planos/ (src/robo/verificar.py).
"""
import subprocess
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

LAB3 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LAB3 / "src" / "robo"))
sys.path.insert(0, str(LAB3 / "src"))
import geometria as G   # noqa: E402

W, H, FPS = 1920, 1080, 30
FONTE = "/System/Library/Fonts/Helvetica.ttc"
AZUL, CINZA, LARANJA = (31, 58, 95), (90, 90, 90), (217, 110, 20)
SCAD = LAB3 / "modelos" / "suporte_imu" / "suporte_imu.scad"
RES = LAB3 / "resultados"

f_tit, f_sub, f_txt, f_peq = (ImageFont.truetype(FONTE, s) for s in (64, 40, 34, 26))


def ffmpeg(out):
    out.parent.mkdir(parents=True, exist_ok=True)
    return subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                             "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264",
                             "-pix_fmt", "yuv420p", "-crf", "20", str(out)], stdin=subprocess.PIPE)


def escrever(ff, img, segundos=None):
    b = np.asarray(img.convert("RGB"), dtype=np.uint8).tobytes()
    for _ in range(int(round((segundos or 1 / FPS) * FPS))):
        ff.stdin.write(b)


def cartao(titulo, linhas, rodape=""):
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    d.text((140, 200), titulo, font=f_tit, fill=AZUL)
    y = 340
    for ln in linhas:
        d.text((140, y), ln, font=f_txt, fill=(30, 30, 30))
        y += 62
    if rodape:
        d.text((140, H - 120), rodape, font=f_peq, fill=CINZA)
    return img


def legenda(img, titulo, sub=""):
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, W, 130 if sub else 90], fill=(255, 255, 255))
    d.text((50, 22), titulo, font=f_sub, fill=AZUL)
    if sub:
        d.text((50, 78), sub, font=f_peq, fill=CINZA)
    return img


# ------------------------------------------------------------------ 1. a peça
def pecas(ff):
    with tempfile.TemporaryDirectory() as tmp:
        for uno, titulo, sub in ((False, "1. A peça: suporte da IMU no lugar de um dedo da pinça",
                                  "Placa da IMU (verde) apertada por 3 M3; só a ficha Grove sai da parede. Torres do Uno a amarelo."),
                                 (True, "1. A peça montada: Arduino Uno (azul) nas torres, por cima da IMU",
                                  "Parede 74 × 59 × 3 mm, sem acrílico. Impressa a 16/11 na Composites.")):
            n = 120
            for k in range(n):
                png = Path(tmp) / f"{int(uno)}_{k:03d}.png"
                az = 300 + 360 * k / n
                subprocess.run(["openscad", "-D", "mostrar_contexto=true", "-D", f"mostrar_uno={str(uno).lower()}",
                                "--imgsize=1600,950", "--projection=p", "--autocenter", "--viewall",
                                f"--camera=0,0,0,105,0,{az:.1f},0", "--colorscheme=Tomorrow",
                                "-o", str(png), str(SCAD)], check=True, capture_output=True)
                img = Image.new("RGB", (W, H), "white")
                img.paste(Image.open(png).convert("RGB"), ((W - 1600) // 2, 130))
                escrever(ff, legenda(img, titulo, sub))


# ------------------------------------------------------------------ 2. ensaios
ENSAIOS = [  # nome, início (s), fim (s), aceleração do vídeo
    ("poses", 0, 70, 7, "calibração do acelerómetro: faces e orientações ao acaso, paradas"),
    ("rotacoes", 0, 40, 4, "rotações conhecidas da junta 6 (escala do giroscópio)"),
    ("inclinacoes", 0, 55, 5, "inclinações de 15 em 15° em torno dos eixos da IMU (Tarefas 3 e 4)"),
    ("abanar", 5, 25, 2, "inclinada a 30° com abanões de ±10 mm a 2 Hz (inclinómetro engana-se)"),
    ("vaivem", 0, 25, 2, "vaivém de ±50 mm (acelerações conhecidas)"),
    ("deriva", 0, 25, 2, "soco de 300 mm e quadrado (deriva da posição, Tarefa 6)"),
    ("acenar", 0, 18, 1.5, "acenar ±30° em torno de um cotovelo (gesto da Tarefa 6)"),
]


def ensaios(ff):
    m, d = G.modelo()
    m.vis.global_.offwidth, m.vis.global_.offheight = W, H
    # dois quadros: o robô inteiro (esquerda) e a pinça com a peça de perto (direita)
    lado, alto = W // 2, H - 130
    r = mujoco.Renderer(m, alto, lado)
    largo, perto = mujoco.MjvCamera(), mujoco.MjvCamera()
    for nome, t0, t1, vel, texto in ENSAIOS:
        z = np.load(LAB3 / "data" / "planos" / f"{nome}.npz")
        t, q = z["t"], z["q"]
        et = [e.split("|") for e in z["etiquetas"]]
        t1 = min(t1, t[-1])
        for ts in np.arange(t0, t1, vel / FPS):
            k = min(int(np.searchsorted(t, ts)), len(t) - 1)
            G.pose_mj(m, d, q[k])
            p = d.site("imu").xpos
            largo.lookat[:] = [0.2, 0.0, 0.25]
            largo.distance, largo.azimuth, largo.elevation = 1.15, 135, -15
            perto.lookat[:] = p
            perto.distance, perto.azimuth, perto.elevation = 0.32, 135, -20
            img = Image.new("RGB", (W, H), "white")
            for j, cam in enumerate((largo, perto)):
                r.update_scene(d, camera=cam)
                img.paste(Image.fromarray(r.render()), (j * lado, 130))
            atual = next((e[2] for e in et if float(e[0]) <= ts < float(e[1])), "")
            legenda(img, f"2. Ensaio no robô (MuJoCo): {nome}  ·  {vel:g}×", texto)
            ImageDraw.Draw(img).text((50, H - 70), f"t = {ts:5.1f} s   {atual}", font=f_txt, fill=(30, 30, 30))
            escrever(ff, img)


# ------------------------------------------------------------------ 3. resultados
FIGURAS = [
    ("sim/t2_modulo.png", "3. Calibração (Tarefa 2), dados simulados",
     "|a| antes e depois de calibrar: recupera desvio < 1 mg e escala < 0,1 %"),
    ("sim/t2_allan.png", "3. Ruído (Tarefa 2): variância de Allan", "ruído e passeio aleatório do giroscópio e do acelerómetro"),
    ("sim/t3_t4_inclinar_abanar.png", "3. Tarefas 3 e 4: inclinómetro e giroscópio",
     "o inclinómetro falha quando há aceleração; o giroscópio integrado deriva"),
    ("sim/t3_t4_robo_inclinacoes.png", "3. As mesmas contas com a verdade do robô simulado",
     "inclinações no robô: o acelerómetro calibrado segue a verdade quando está parado"),
]


def resultados(ff):
    for rel, titulo, sub in FIGURAS:
        f = RES / rel
        if not f.exists():
            continue
        fig = Image.open(f).convert("RGB")
        esc = min((W - 100) / fig.width, (H - 180) / fig.height)
        fig = fig.resize((int(fig.width * esc), int(fig.height * esc)), Image.LANCZOS)
        img = Image.new("RGB", (W, H), "white")
        img.paste(fig, ((W - fig.width) // 2, 150))
        escrever(ff, legenda(img, titulo, sub), 4.5)


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else LAB3 / "video" / "media" / "estado_lab3.mp4"
    ff = ffmpeg(out)
    escrever(ff, cartao("Lab 3: IMU — ponto de situação (18/11)", [
        "• Sketch do Arduino em bruto a 200 Hz e gravador da porta série",
        "• Simulador da IMU com erros conhecidos, para validar a análise",
        "• Calibração (Tarefa 2), inclinómetro e giroscópio (Tarefas 3 e 4)",
        "• Suporte da IMU para a pinça, redesenhado com as medidas de 16/11",
        "• Sete ensaios do robô verificados no MuJoCo, com a peça montada",
    ], "Grupo: Afonso Cruz, Henrique Póvoa, Eduardo Batista"), 5)
    pecas(ff)
    ensaios(ff)
    resultados(ff)
    escrever(ff, cartao("O que falta", [
        "• Verificar a peça impressa e montar a IMU e o Uno",
        "• Gravar os ensaios à mão (A a F) e calibrar com dados reais",
        "• Ensaios no robô (23–29/11): verdade para as Tarefas 2 a 4",
        "• Tarefa 5 (fusão, peso fixo e dinâmico) e Tarefa 6 (gestos)",
        "• Relatório e entrega a 18/12",
    ]), 5)
    ff.stdin.close()
    ff.wait()
    print(out)


if __name__ == "__main__":
    main()
