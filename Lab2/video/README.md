# Vídeo dos ensaios

Vídeo publicado: <https://youtu.be/F_t07cNw1qI>. Estes scripts montaram-no a
partir dos vídeos do telemóvel e dos dados gravados em `../data/`.

| Script | O que faz |
|---|---|
| `video_cartoes.py` | cartões de texto entre ensaios (o inicial usa `video_endereco.jpg`) |
| `video_mujoco.py` | clip da simulação no MuJoCo, com as folgas em cada instante |
| `video_dados.py` | junta o vídeo de um ensaio com os dados desse ensaio (juntas, velocidade, erro) |
| `video_som.py` | som gerado a partir da velocidade medida, e mistura final |

`media/` tem os vídeos originais, as versões montadas e a música
("Deliberate Thought", Kevin MacLeod, CC BY 4.0). Não vai para o git.
Precisa de `ffmpeg`. Os instantes de sincronização de cada ensaio estão em
`video_som.py` (`ENSAIOS`).
