# Gripper Lite (pinça do Lite 6) — medidas tiradas do STEP do fabricante

Fonte: `Gripper Lite.STEP`, descarregado da página de ficheiros 3D da UFACTORY
(13/11/2026). Um só sólido, unidades em mm. Eixos: z ao longo da ferramenta
(0 no flange), y na direção em que os dedos abrem, x a largura dos dedos.

| | mm |
|---|---|
| Altura total, do flange à ponta dos dedos | 81,1 |
| Corpo: diâmetro | 77,5 |
| Face inferior do corpo | z ≈ 57,6 |
| Dedos abaixo do corpo | 23,5 |
| Lâmina de cada dedo: espessura | 4,2 |
| Lâmina: largura | 20 na ponta e na base, 17 a meio |
| Lâmina: altura livre da face interior | 16,5 |
| Curso de cada dedo | 8,1 (total 16,2) |
| Espaço entre dedos, aberta (como desenhado) | 38,4 |
| Espaço entre dedos, fechada | 22,2 |
| De fora a fora, aberta / fechada | 46,8 / 30,6 |
| Fixação de cada dedo | 2 parafusos M2,5 verticais, a 12 mm um do outro (x = ±6), com caixa Ø 5 |

Os dedos são em L. No ficheiro estão montados com a lâmina do lado de fora,
que é a montagem que deixa espaço com a pinça fechada (22,2 a 38,4 mm).
Virando os dedos, a lâmina fica do lado de dentro e o intervalo passa a
0 a 16,2 mm; é essa a variante do modelo MuJoCo usado no Lab 2 ("narrow").

Por confirmar no robô: qual das duas montagens está no laboratório (as fotos
apontam para a de fora) e se o curso real é mesmo 16,2 mm.

Força de aperto: 5 N, sem realimentação (abre/fecha).
