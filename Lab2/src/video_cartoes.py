"""
Cartões de texto (separadores) para o vídeo dos ensaios: 1920x1080, alguns segundos cada.

Uso (a partir de Lab2/):
    python src/video_cartoes.py PASTA      # escreve PASTA/k0.mp4 ... k5.mp4
"""
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

W, H, FPS = 1920, 1080, 30
VERDE, AZUL, CINZA = (27, 127, 59), (31, 58, 95), (82, 81, 78)
F = "/System/Library/Fonts/Helvetica.ttc"

CARTOES = [
    ("k1", 5, "1. Slow first", "Half the planned speed",
     ["Does the robot follow the trajectory?",
      "On the right: what was planned for the joints and what the robot measured.",
      "",
      "Result: 0.56 mm mean error — and it is just a 10 ms lag."]),
    ("k2", 7, "2. At the planned speed", "Twice as fast: 215 mm/s at the gripper",
     ["Run 1 suggests the robot is simply 10 ms late (one control cycle).",
      "If so, the error must double when the speed doubles.",
      "",
      "Prediction:  215 mm/s × 0.010 s = 2.2 mm maximum.",
      "Measured: 2.16 mm."]),
    ("k3", 9, "3. Faster than planned", "306 mm/s at the gripper — 43 % above the plan",
     ["Why push it? Our first swing design could not keep up with its own plan:",
      "the robot fell 8 mm behind at only 170 mm/s.",
      "",
      "The swing was redesigned joint by joint. Does the new one hold beyond its plan?",
      "",
      "If the only error is the 10 ms lag:  306 mm/s × 0.010 s = 3.1 mm.",
      "Measured: 3.09 mm.  It holds."]),
    ("k4", 5, "4. And with a real ball?", "A pen cap on top of an eraser",
     ["The gripper was moved by hand to the cap and the joints were read:",
      "that pose is the new address. The rest of the swing is the same.",
      "",
      "The tip passes where it was planned. (A light cap on purpose: it is a demo.)"]),
    ("k5", 8, "What the data say", "",
     ["• The tracking error is speed × 10 ms: one control cycle of lag, nothing else.",
      "• With the lag removed, less than 0.05 mm is left.",
      "• Predicted and measured agree at all three speeds.",
      "• The simulation predicted 25 mm between links and 82 mm to the table: no contact.",
      "",
      "Error is relative to the pose reported by the controller (no external measurement)."]),
]


def cartao(titulo, sub, linhas):
    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, 28, H], fill=VERDE)
    d.text((140, 200), titulo, font=ImageFont.truetype(F, 84), fill=VERDE)
    d.text((140, 310), sub, font=ImageFont.truetype(F, 46), fill=AZUL)
    d.line([(140, 395), (W - 140, 395)], fill=VERDE, width=4)
    y = 460
    for l in linhas:
        d.text((140, y), l, font=ImageFont.truetype(F, 42), fill=(11, 11, 11) if not l.startswith("Error is relative") else CINZA)
        y += 78
    return img


FOTO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "report", "video_endereco.jpg")
# (texto, explicação, ponto na fotografia [px da foto de 880x940])
LEGENDA = [
    ("Joint 3 = elbow", "fixed at 80°", (330, 220)),
    ("Joint 5 = wrist", "cocks on the way up, releases before impact", (190, 430)),
    ("Joint 2 = shoulder", "raises and lowers the arm", (590, 505)),
    ("Gripper tip = club head", "passes through the ball at top speed", (206, 560)),
    ("Joint 1 (base) = torso", "turns the whole arm: the engine of the swing", (564, 620)),
]


def cartao_intro():
    """Cartão inicial: fotografia do robô no endereço com legendas ligadas a cada junta."""
    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, 28, H], fill=VERDE)
    foto = Image.open(FOTO)
    fx, fy = 80, 70
    img.paste(foto, (fx, fy))
    x0 = fx + foto.width + 110
    d.text((x0, 70), "Golf swing with the", font=ImageFont.truetype(F, 62), fill=VERDE)
    d.text((x0, 140), "UFactory Lite 6", font=ImageFont.truetype(F, 62), fill=VERDE)
    d.text((x0, 222), "Robotics — Lab assignment 2 — Group 1 (IST)", font=ImageFont.truetype(F, 30), fill=AZUL)
    d.line([(x0, 275), (W - 90, 275)], fill=VERDE, width=4)
    y = 330
    for texto, expl, (px, py) in LEGENDA:
        alvo = (fx + px, fy + py)
        d.line([alvo, (x0 - 24, y + 24)], fill=(255, 255, 255), width=7)
        d.line([alvo, (x0 - 24, y + 24)], fill=AZUL, width=3)
        d.ellipse([alvo[0] - 11, alvo[1] - 11, alvo[0] + 11, alvo[1] + 11], fill=AZUL, outline=(255, 255, 255), width=3)
        d.ellipse([x0 - 32, y + 16, x0 - 16, y + 32], fill=AZUL)
        d.text((x0, y), texto, font=ImageFont.truetype(F, 40), fill=(11, 11, 11))
        d.text((x0, y + 50), expl, font=ImageFont.truetype(F, 27), fill=CINZA)
        y += 118
    d.text((x0, 930), "Planned and simulated first.", font=ImageFont.truetype(F, 30), fill=AZUL)
    d.text((x0, 972), "Then: does the robot do what was predicted?", font=ImageFont.truetype(F, 30), fill=AZUL)
    return img


def main():
    pasta = sys.argv[1]
    os.makedirs(pasta, exist_ok=True)
    feitos = [("k0", 8, "intro", cartao_intro())] + [(n, sg, t, cartao(t, sb, l)) for n, sg, t, sb, l in CARTOES]
    for nome, seg, titulo, imagem in feitos:
        png = os.path.join(pasta, nome + ".png")
        imagem.save(png)
        subprocess.check_call(["ffmpeg", "-v", "error", "-y", "-loop", "1", "-t", str(seg), "-i", png, "-r", str(FPS),
                               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", os.path.join(pasta, nome + ".mp4")])
        print(f"  {nome}: {seg} s  {titulo}")


if __name__ == "__main__":
    main()
