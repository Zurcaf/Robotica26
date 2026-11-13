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
    ("k0", 6, "Golf swing with the UFactory Lite 6", "Robotics — Lab assignment 2 — Group 1 (IST)",
     ["The robot plays the golfer and the gripper plays the club.",
      "Base = torso      Joint 2 = shoulder      Joint 5 = wrist",
      "",
      "The swing was planned and simulated before going to the robot.",
      "The question in every run: does the robot do what was predicted?"]),
    ("k1", 5, "1. Slow first", "Half the planned speed",
     ["Does the robot follow the trajectory?",
      "On the right: what was planned for the joints and what the robot measured.",
      "",
      "Result: 0.56 mm mean error — and it is just a 10 ms lag."]),
    ("k2", 5, "2. At the planned speed", "Twice as fast",
     ["If the error is one control cycle of lag (10 ms), it must double with speed.",
      "",
      "Predicted: 2.2 mm maximum.      Measured: 2.16 mm."]),
    ("k3", 5, "3. Faster than planned", "306 mm/s — 43 % above the plan",
     ["A first version of the swing failed at 170 mm/s (8 mm error).",
      "This one, designed in joint space, still follows at 306 mm/s.",
      "",
      "Predicted: 3.1 mm maximum.      Measured: 3.09 mm."]),
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


def main():
    pasta = sys.argv[1]
    os.makedirs(pasta, exist_ok=True)
    for nome, seg, titulo, sub, linhas in CARTOES:
        png = os.path.join(pasta, nome + ".png")
        cartao(titulo, sub, linhas).save(png)
        subprocess.check_call(["ffmpeg", "-v", "error", "-y", "-loop", "1", "-t", str(seg), "-i", png, "-r", str(FPS),
                               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", os.path.join(pasta, nome + ".mp4")])
        print(f"  {nome}: {seg} s  {titulo}")


if __name__ == "__main__":
    main()
