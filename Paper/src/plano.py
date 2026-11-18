"""Plano do bico no chão (bico sempre para baixo): zona útil e caminho suave.

Regras (pedido de 17/11): pode abrandar, mas tem de ser suave e sem colisões.
- Configuração fixa S = 0 (braço à frente do A1, cotovelo esticado, A5 > 0): é a
  única que chega por baixo do robô; mudar de configuração obriga a passar por
  uma singularidade.
- Margens às singularidades: A5 >= MARGEM_A5 (punho dobrado) e o cotovelo pelo
  menos MARGEM_A3 longe de esticado. Escolha nossa, por afinar.
- Sem colisões (robô e extrusora com o chão e consigo próprios, ver cena.py).
- A rotação da extrusora em torno do bico (psi) é livre: escolhe-se para o
  caminho todo de uma vez (programação dinâmica), sem nenhuma junta a saltar
  mais de DQ_MAX entre pontos seguidos, e, entre as que servem, a mais rápida.
"""
from __future__ import annotations

import numpy as np

import cena
import kuka as K

S_FIXO = 2             # A3 "para baixo" (cotovelo normal, a do backup): chega ao pé da parede de cima
MARGEM_A5 = 15.0       # °, |A5| mínimo (singularidade do punho em A5 = 0)
MARGEM_A3 = 10.0       # °, distância mínima do cotovelo à posição esticada
DQ_MAX = 10.0          # °, maior salto de qualquer junta entre pontos seguidos
PASSO_PT = 20.0        # mm entre pontos ao longo do caminho
PASSO_EIXO = 5.0       # mm entre pontos perto do eixo do A1 (aí o A1 roda depressa)
R_EIXO = 600.0         # mm, raio à volta do eixo onde se usa PASSO_EIXO
PSI = np.radians(np.arange(-180, 180, 5.0))


def ik_s0(p_bico_w, psi):
    """Inversa vetorizada na configuração S_FIXO (0 ou 2), bico para baixo.
    p_bico_w (N,3) no mundo, mm; psi em rad. Devolve q (N,6) em graus, no
    intervalo (-180, 180], e válido (N,) bool (alcance, limites e margens)."""
    T_rw = np.linalg.inv(K.robo_no_mundo())
    xf = np.array([0.0, 0, -1])
    zf = np.array([np.cos(psi), np.sin(psi), 0.0])
    Rw = np.column_stack([xf, np.cross(zf, xf), zf])
    R = T_rw[:3, :3] @ Rw
    p_fl = (p_bico_w - Rw @ K.frame(*K.TOOL_BICO)[:3, 3]) @ T_rw[:3, :3].T + T_rw[:3, 3]
    pw = p_fl - K.D6 * R[:, 2]
    L3e = np.hypot(K.L3, K.A3_OFF)
    delta = np.arctan2(K.SINAL_55 * K.A3_OFF, K.L3)
    rho = np.hypot(pw[:, 0], pw[:, 1])
    a1 = -np.arctan2(pw[:, 1], pw[:, 0])
    r, h = rho - K.A1_OFF, pw[:, 2] - K.D1
    D = (r * r + h * h - K.L2 ** 2 - L3e ** 2) / (2 * K.L2 * L3e)
    g = np.arccos(np.clip(D, -1, 1))                      # g > 0  <=>  bit 1 do S = 0
    if S_FIXO & 2:                                        # bit 1 = 1: o outro cotovelo
        g = -g
    e2 = np.arctan2(h, r) - np.arctan2(L3e * np.sin(g), K.L2 + L3e * np.cos(g))
    a2 = (-e2 + np.pi) % (2 * np.pi) - np.pi
    a3 = (delta - g + np.pi) % (2 * np.pi) - np.pi
    c1, s1 = np.cos(-a1), np.sin(-a1)
    c23, s23 = np.cos(a2 + a3), np.sin(a2 + a3)
    R03 = np.zeros((len(pw), 3, 3))
    R03[:, 0] = np.stack([c1 * c23, -s1, c1 * s23], 1)
    R03[:, 1] = np.stack([s1 * c23, c1, s1 * s23], 1)
    R03[:, 2] = np.stack([-s23, np.zeros_like(s23), c23], 1)
    M = np.einsum("nji,jk->nik", R03, R @ K.R_F0.T)       # R03^T R R_F0^T
    a5 = np.arccos(np.clip(M[:, 0, 0], -1, 1))            # A5 > 0  <=>  bit 2 do S = 0
    a = np.arctan2(M[:, 1, 0], -M[:, 2, 0])
    c = np.arctan2(M[:, 0, 1], M[:, 0, 2])
    q = np.degrees(np.stack([a1, a2, a3, -a, a5, -c], 1))
    q = (q + 180) % 360 - 180
    lim = K.LIM
    ok = ((np.abs(D) <= 1) & (q[:, 1] >= lim[1, 0]) & (q[:, 1] <= lim[1, 1])
          & (q[:, 2] >= lim[2, 0]) & (q[:, 2] <= lim[2, 1])
          & (np.abs(np.degrees(g)) >= MARGEM_A3) & (q[:, 4] >= MARGEM_A5) & (q[:, 4] <= lim[4, 1]))
    return q, ok


def zona(m, d, z=0.0, passo=25.0):
    """Pontos do chão com pelo menos uma rotação da extrusora válida e sem colisões."""
    xs = np.arange(passo / 2, K.SALA[0], passo)
    ys = np.arange(passo / 2, K.SALA[1], passo)
    GX, GY = np.meshgrid(xs, ys)
    p = np.column_stack([GX.ravel(), GY.ravel(), np.full(GX.size, z)])
    Q, V = zip(*(ik_s0(p, psi) for psi in PSI))
    Q, V = np.stack(Q, 1), np.stack(V, 1)                # (N, G, 6), (N, G)
    ok = np.zeros(len(p), bool)
    for i in np.flatnonzero(V.any(1)):
        for g in np.flatnonzero(V[i]):
            if not cena.colisoes(m, d, Q[i, g]):
                ok[i] = True
                break
    return xs, ys, ok.reshape(GX.shape)


def _segmentos(fila, xs):
    idx = np.flatnonzero(fila)
    if len(idx) == 0:
        return []
    return [(xs[t[0]], xs[t[-1]]) for t in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)]


def _celulas(xs, ys, M, ylin):
    """Divide a zona em células que se varrem de seguida (decomposição
    boustrophedon): uma célula continua enquanto cada linha tiver um só troço
    que se sobrepõe ao da linha anterior. Devolve listas de (y, x0, x1)."""
    passo_y = ys[1] - ys[0]
    abertas, fechadas = [], []                     # cada célula: lista de (y, x0, x1)
    for y in ylin:
        segs = _segmentos(M[int(round((y - ys[0]) / passo_y))], xs)
        novas = []
        for x0, x1 in segs:
            cont = [c for c in abertas if not (x1 < c[-1][1] or x0 > c[-1][2])]
            if len(cont) == 1 and sum(not (c[-1][2] < x0 or c[-1][1] > x1) for c in abertas) == 1 \
                    and sum(not (x1b < cont[0][-1][1] or x0b > cont[0][-1][2]) for x0b, x1b in segs) == 1:
                cont[0].append((y, x0, x1))
                novas.append(cont[0])
            else:
                novas.append([(y, x0, x1)])
        fechadas += [c for c in abertas if not any(c is n for n in novas)]
        abertas = novas
    return fechadas + abertas


def _ligar(a, b, xs, ys, M, z):
    """Caminho pelo chão de a até b sem sair da zona: menor caminho na grelha
    (8 vizinhos) e depois atalhos em linha reta que fiquem dentro da zona."""
    from collections import deque
    passo = xs[1] - xs[0]
    ij = lambda p: (int(round((p[1] - ys[0]) / passo)), int(round((p[0] - xs[0]) / passo)))
    ini, fim = ij(a), ij(b)
    prev = {ini: None}
    fila = deque([ini])
    while fila:
        c = fila.popleft()
        if c == fim:
            break
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                n = (c[0] + di, c[1] + dj)
                if n not in prev and 0 <= n[0] < M.shape[0] and 0 <= n[1] < M.shape[1] and M[n]:
                    prev[n] = c
                    fila.append(n)
    if fim not in prev:
        return None
    nos, c = [], fim
    while c is not None:
        nos.append(np.array([xs[c[1]], ys[c[0]], z]))
        c = prev[c]
    nos = [np.asarray(a, float)] + nos[::-1][1:-1] + [np.asarray(b, float)]

    def dentro(p, q):
        n = int(np.linalg.norm(q - p) / (passo / 2)) + 2
        return all(M[ij(p + (q - p) * f)] for f in np.linspace(0, 1, n))

    simples, i = [nos[0]], 0
    while i < len(nos) - 1:
        j = len(nos) - 1
        while j > i + 1 and not dentro(nos[i], nos[j]):
            j -= 1
        simples.append(nos[j])
        i = j
    return simples


def _reta(p, q):
    """Pontos de p até q (sem p), de PASSO_PT em PASSO_PT mm, ou PASSO_EIXO perto do eixo."""
    out, d = [], np.linalg.norm(q - p)
    s = 0.0
    while True:
        perto = np.hypot(*(p + (q - p) * min(s / d, 1) - [*K.ROBO_XY, 0])[:2]) < R_EIXO if d else False
        s += PASSO_EIXO if perto else PASSO_PT
        if s >= d:
            out.append(q.copy())
            return out
        out.append(p + (q - p) * s / d)


def linhas(xs, ys, M, entre, z=0.0):
    """Zigue-zague ao longo de x, uma linha a cada `entre` mm, célula a célula.
    Dentro de uma célula as linhas ligam-se pela ponta; entre células o bico anda
    pelo chão sem sair da zona. No início de cada linha fica parado um bocado
    (36 pontos iguais), para a extrusora poder rodar no sítio."""
    ylin = np.arange(ys[M.any(1)].min(), ys[M.any(1)].max() + 1, entre)
    cels = _celulas(xs, ys, M, ylin)
    pts = []
    for cel in cels:
        for k, (y, x0, x1) in enumerate(cel):
            if pts:                                       # começar pela ponta mais perto
                if k == 0 and abs(pts[-1][0] - x1) < abs(pts[-1][0] - x0):
                    cel = [(yy, b, a) for yy, a, b in cel]
                    y, x0, x1 = cel[0]
                if k > 0 and (k % 2) != (0 if cel[0][1] <= cel[0][2] else 1):
                    pass
            if k % 2:
                x0, x1 = x1, x0
            ini = np.array([x0, y, z])
            if pts:
                cam = _ligar(pts[-1], ini, xs, ys, M, z) or [pts[-1], ini]
                for p, q in zip(cam[:-1], cam[1:]):
                    pts += _reta(np.asarray(p, float), np.asarray(q, float))
                pts += [ini] * 36
            else:
                pts.append(ini)
            pts += _reta(ini, np.array([x1, y, z]))
    return np.array(pts)


def _dif(a, b):
    return (a - b + 180) % 360 - 180


def planear(pts, m, d):
    """Programação dinâmica sobre a rotação da extrusora e as voltas do A4 e do A6.

    Os estados de cada ponto são (psi, volta do A4, volta do A6) com os ângulos
    absolutos dentro dos limites; o A1 fica no intervalo (-180, 180] e não pode
    passar por ±180 (só vai a ±185). Transição permitida se nenhuma junta andar
    mais de DQ_MAX; custo = tempo da junta mais lenta. Devolve as juntas (P,6) e
    os índices onde o caminho parte (sem transição válida)."""
    P, G = len(pts), len(PSI)
    Q0 = np.zeros((P, G, 6))
    V0 = np.zeros((P, G), bool)
    for g, psi in enumerate(PSI):
        Q0[:, g], V0[:, g] = ik_s0(pts, psi)
    for i in range(P):                                    # colisões só nos candidatos
        for g in np.flatnonzero(V0[i]):
            if cena.colisoes(m, d, Q0[i, g]):
                V0[i, g] = False
    voltas = [(r4, r6) for r4 in (-360.0, 0.0, 360.0) for r6 in (-360.0, 0.0, 360.0)]
    estados, qest = [], []                                # por ponto: índices g e juntas de cada estado
    for i in range(P):
        gs = np.flatnonzero(V0[i])
        qq = [Q0[i, g] + [0, 0, 0, r4, 0, r6] for g in gs for r4, r6 in voltas]
        ok = [K.LIM[3, 0] <= q[3] <= K.LIM[3, 1] and K.LIM[5, 0] <= q[5] <= K.LIM[5, 1] for q in qq]
        qest.append(np.array([q for q, o in zip(qq, ok) if o]).reshape(-1, 6))
    custo = np.zeros(len(qest[0]))
    ant, partes = [None], []
    for i in range(1, P):
        a, b = qest[i - 1], qest[i]
        if len(a) == 0 or len(b) == 0:
            partes.append(i)
            ant.append(None)
            custo = np.zeros(len(b))
            continue
        dq = np.abs(b[:, None, :] - a[None, :, :])        # (novo, antigo, 6), ângulos absolutos
        t = np.max(dq / K.VEL, axis=2)
        t[(dq > DQ_MAX).any(2)] = np.inf
        tot = custo[None, :] + t
        k = np.argmin(tot, axis=1)
        novo = tot[np.arange(len(b)), k]
        if not np.isfinite(novo).any():                   # parte: recomeça aqui
            partes.append(i)
            ant.append(None)
            custo = np.zeros(len(b))
            continue
        ant.append(k)
        custo = novo
    q = np.zeros((P, 6))
    e = int(np.argmin(custo))
    for i in range(P - 1, -1, -1):
        q[i] = qest[i][e] if len(qest[i]) else np.nan
        if i > 0:
            e = int(ant[i][e]) if ant[i] is not None else int(np.argmin(np.zeros(len(qest[i - 1])) if len(qest[i - 1]) else [0]))
    return q, partes, None


def verificar(qs, pts, m, d, passo=1.0):
    """Saltos, limites, bico no sítio e colisões nos movimentos entre pontos."""
    T = K.robo_no_mundo()
    bom = ~np.isnan(qs).any(1)
    erro = max(np.linalg.norm((T @ K.fk_bico(q))[:3, 3] - p) for q, p in zip(qs[bom], pts[bom]))
    dq = np.abs(np.diff(qs, axis=0))
    salto = np.nanmax(dq)
    fora = ((qs[bom] < K.LIM[:, 0] - 1e-6) | (qs[bom] > K.LIM[:, 1] + 1e-6)).any(0)
    maus = []
    for i in range(len(qs) - 1):
        if not (bom[i] and bom[i + 1]):
            continue
        n = max(1, int(np.ceil(np.abs(qs[i + 1] - qs[i]).max() / passo)))
        for f in np.linspace(0, 1, n + 1):
            c = cena.colisoes(m, d, qs[i] + (qs[i + 1] - qs[i]) * f)
            if c:
                maus.append((i, c))
                break
    return dict(erro=erro, salto=salto, fora=[f"A{j + 1}" for j in np.flatnonzero(fora)], colisoes=maus,
                status=sorted({K.status(q) for q in qs[bom]}), a5_min=qs[bom, 4].min(),
                sem_pose=int((~bom).sum()))
