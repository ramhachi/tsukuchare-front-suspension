"""Task 4: パラメータスイープ + 制約フィルタ + Pareto 出力
R=50 (径100mm) 確定。段差は h_cap (乗越え可能最大段差) で評価。
- h_cap_rigid: リジッド理想円の必要推力が摩擦限界以下になる最大 h (保守的下限)
- h_cap_soft : suspension-assist (ばねが荷重を担う準静的下限) での最大 h (楽観上限)
実行: python3 optimize_geometry.py → pareto_candidates.csv
"""
import csv
import itertools
import numpy as np

g = 9.81
M_TOTAL = 20.0
R = 50.0                        # 確定: 径100mm オムニ
ALPHA_MAX = np.radians(40)
MU_REAR = 0.8
REAR_RATIO = 0.65               # 保守的な仮置き
SAG_TARGET = 0.25
GENERIC_STROKE = 40.0
H_GRID = np.arange(10, 71, 2.5) # 10-70mm を評価

A_LIST = [50.0, 70.0, 90.0, 110.0]
BA_LIST = [0.4, 0.6, 0.8, 1.0, 1.2]
RHO_LIST = [0.5, 0.7, 0.9]
XU_LIST = [-40.0, -20.0, 0.0]
ZU_LIST = [20.0, 40.0, 60.0]

N_front = M_TOTAL * g * (1 - REAR_RATIO) / 2
F_friction = MU_REAR * M_TOTAL * g * REAR_RATIO

def h_caps():
    """段差ごとの必要推力 (リジッド / soft) から h_cap を返す"""
    rigid, soft = 0.0, 0.0
    for h in H_GRID:
        if h >= 0.95 * R:
            break
        s = np.sqrt(2 * R * h - h**2)
        if N_front * s / (R - h) < F_friction:
            rigid = h
        if N_front * s / R < F_friction:
            soft = h
    return rigid, soft

def evaluate(a, b, rho, xu, zu):
    U = (xu, zu)
    def wpos(al):
        return np.array([-a * np.cos(al) - b * np.sin(al),
                         a * np.sin(al) - b * np.cos(al)])
    def dz_dal(al):
        return np.array([a * np.sin(al) - b * np.cos(al),
                         a * np.cos(al) + b * np.sin(al)])
    def ell(al):
        return np.linalg.norm(rho * wpos(al) - np.array(U))
    def MR(al):
        dS = rho * dz_dal(al)
        dl = np.dot(rho * wpos(al) - np.array(U), dS) / ell(al)
        return abs(dl / dz_dal(al)[1])
    dz_max = wpos(ALPHA_MAX)[1] - wpos(0)[1]
    dx_max = wpos(ALPHA_MAX)[0] - wpos(0)[0]
    h_rigid, h_soft = h_caps()
    # Cq は現実的な範囲 [20, min(40, h_soft)] で最小値
    hs = np.linspace(20, max(20, min(40, h_soft)), 5)
    cqs = []
    for h in hs:
        s = np.sqrt(2 * R * h - h**2)
        n = np.array([-s, R - h]) / R
        t = np.array([-b, a]) / np.hypot(a, b)
        cqs.append(np.dot(t, n))
    Cq_min = min(cqs)
    k_s = N_front / (SAG_TARGET * dz_max) / MR(0.0)**2
    mrs = np.array([MR(x) for x in np.linspace(0, ALPHA_MAX, 20)])
    ells = np.array([ell(x) for x in np.linspace(0, ALPHA_MAX, 20)])
    stroke_needed = ells[0] - ells[-1]
    F_shock_max = k_s * stroke_needed
    fails = []
    if dz_max < 50:              # 段差40 + 余裕10
        fails.append(f"travel {dz_max:.0f}<50")
    if Cq_min < 0.85:
        fails.append(f"Cq {Cq_min:.2f}")
    if not (0.15 <= mrs.min() <= 1.2):
        fails.append("MR range")
    if stroke_needed > GENERIC_STROKE:
        fails.append(f"stroke {stroke_needed:.0f}")
    if ells.min() < 40:
        fails.append("shock min len")
    if np.isnan(F_shock_max) or F_shock_max > 800:
        fails.append(f"F_shock {F_shock_max:.0f}")
    return dict(a=a, b=b, rho=rho, xu=xu, zu=zu, N_front=round(N_front, 1),
                k_spring=round(k_s, 2), dz_max=round(dz_max, 1),
                dx_max=round(dx_max, 1), Cq_min=round(Cq_min, 3),
                MR_min=round(mrs.min(), 3), MR_max=round(mrs.max(), 3),
                stroke_needed=round(stroke_needed, 1),
                F_shock_max=round(F_shock_max, 1),
                h_cap_rigid=round(h_rigid, 1), h_cap_soft=round(h_soft, 1),
                feasible=len(fails) == 0, failed=";".join(fails))

rows = []
for a, ba, rho, xu, zu in itertools.product(A_LIST, BA_LIST, RHO_LIST, XU_LIST, ZU_LIST):
    b = a * ba
    if rho * np.hypot(a, b) < 20:
        continue
    rows.append(evaluate(a, b, rho, xu, zu))

feas = [r for r in rows if r["feasible"]]
print(f"全候補 {len(rows)}, 制約通過 {len(feas)}")
if feas:
    print(f"h_cap_rigid 最大 {max(r['h_cap_rigid'] for r in feas):.0f}mm / "
          f"h_cap_soft 最大 {max(r['h_cap_soft'] for r in feas):.0f}mm")

def dominates(x, y):
    key = lambda r: (-r["Cq_min"], r["F_shock_max"], r["stroke_needed"])
    a1, a2 = key(x), key(y)
    return all(p <= q for p, q in zip(a1, a2)) and a1 != a2
pareto = [r for r in feas if not any(dominates(o, r) for o in feas)]
pareto.sort(key=lambda r: -r["Cq_min"])

cols = ["a", "b", "rho", "xu", "zu", "N_front", "k_spring", "dz_max", "dx_max",
        "Cq_min", "MR_min", "MR_max", "stroke_needed", "F_shock_max",
        "h_cap_rigid", "h_cap_soft", "feasible", "failed"]
with open("pareto_candidates.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=cols)
    w.writeheader()
    w.writerows(rows)
print(f"saved pareto_candidates.csv (Pareto {len(pareto)} 件)")
print("\n--- Pareto 上位 10 (Cq_min 降順) ---")
for r in pareto[:10]:
    print(f"a={r['a']:.0f} b={r['b']:.0f} rho={r['rho']} U=({r['xu']:.0f},{r['zu']:.0f}) | "
          f"Cq={r['Cq_min']:.2f} travel={r['dz_max']:.0f}mm stroke={r['stroke_needed']:.0f}mm "
          f"k={r['k_spring']:.1f} F_shock={r['F_shock_max']:.0f}N "
          f"h_cap={r['h_cap_rigid']:.0f}〜{r['h_cap_soft']:.0f}mm")
