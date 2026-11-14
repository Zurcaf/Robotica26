# report/ — relatório do Lab 3

`ROB_LAB3_Report/` segue o formato dos Labs 1 e 2: `main.tex` (preâmbulo,
capa e resumo), uma secção por ficheiro em `secoes/`, `refs.bib` e `Photos/`.

```bash
cd report/ROB_LAB3_Report && latexmk -pdf main.tex && latexmk -c
```

Estado: **esqueleto**. Os modelos e os métodos estão escritos; os resultados
estão marcados a vermelho com `\todo{...}` e as figuras aparecem como caixas
"figura em falta" até existirem em `Photos/`.

| Ficheiro | Conteúdo | Figuras previstas (em `Photos/`) |
|---|---|---|
| `montagem.tex` | sensor, aquisição, robô como referência, tabela dos ensaios | `montagem.png` |
| `t1.tex` | Tarefa 1: modelos e parâmetros a estimar | — |
| `t2.tex` | Tarefa 2: ruído, calibração, temperatura | `ruido.png`, `calib_acc.png`, `calib_gyro.png` |
| `t3.tex` | Tarefa 3: inclinómetro | (usa `fusao_tempo.png`) |
| `t4.tex` | Tarefa 4: giroscópio, quaterniões | (usa `fusao_tempo.png`) |
| `t5.tex` | Tarefa 5: CI; comparação com Kalman (opcional) | `fusao_tempo.png`, `fusao_w.png` |
| `t6.tex` | Tarefa 6: da orientação à posição, validação no robô, gestos à mão | `deriva.png`, `gestos.png` |
| `extra.tex` | controlar o robô com a mão (opcional) | — |
| `comentario.tex` | comentário, contribuições, declaração de IA | — |
| `videos.tex` | ligações | — |

A bibliografia foi escrita de memória: confirmar cada entrada em fonte
primária antes de entregar.
