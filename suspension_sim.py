"""つくばチャレンジ 前輪 trailing-arm サスペンション 2D シミュレータ
計画書 Task 1 (kinematics) + Task 2 (shock/MR) の最小実装。
実行: python3 suspension_sim.py
"""
import numpy as np
import matplotlib.pyplot as plt

# ---- params (params.csv から読み込み。TBD は下のデフォルトを使う) ----
import csv, os

_defaults = dict(R=100.0, H=30.0, a=80.0, b=40.0, rho=0.7,
                 k_spring=20.0, preload=0.0, Fw_static=250.0,
                 m_corner=None)
_params = {k: v for k, v in _defaults.items()}
_alpha_max_deg = 40.0
if os.path.exists("params.csv"):
    with open("params.csv") as f:
        for row in csv.DictReader(l for l in f if not l.startswith("#")):
            if row["value"] in ("", "TBD"):
                continue
            k, v = row["name"], row["value"]
            m = {"R": "R", "h_target_min": "H", "a": "a", "b": "b",
                 "k_spring": "k_spring", "preload": "preload",
                 "front_static_load": "Fw_static", "m_corner": "m_corner",
                 "rho": "rho", "x_u": "xu", "z_u": "zu"}
            if k in m:
                _params[m[k]] = float(v)
            elif k == "alpha_max":
                _alpha_max_deg = float(v)

R = _params["R"]; H = _params["H"]; a = _params["a"]; b = _params["b"]
rho = _params["rho"]; k_spring = _params["k_spring"]
preload = _params["preload"]; Fw_static = _params["Fw_static"]
m_corner = _params["m_corner"]
alpha_max = np.radians(_alpha_max_deg)
U = (_params.get("xu", -20.0), _params.get("zu", 20.0))
if m_corner is None:
    m_corner = Fw_static / 9.81 * 1.5  # TBD 時の粗い代替

def wheel_pos(al):
    return np.array([-a * np.cos(al) - b * np.sin(al),
                     a * np.sin(al) - b * np.cos(al)])

def dq_dal(al):
    return np.array([a * np.sin(al) - b * np.cos(al),
                     a * np.cos(al) + b * np.sin(al)])

def shock_len(al):
    S = rho * wheel_pos(al)
    return np.linalg.norm(S - np.array(U))

def MR(al):
    S = rho * wheel_pos(al)
    dS = rho * dq_dal(al)
    dl = np.dot(S - np.array(U), dS) / shock_len(al)
    return abs(dl / dq_dal(al)[1])

def Cq(h):
    """初期接触反力と初期バンプ方向の結合指標 (§5)"""
    s = np.sqrt(2 * R * h - h**2)
    n = np.array([-s, R - h]) / R
    t = np.array([-b, a]) / np.hypot(a, b)
    return np.dot(t, n)

def b_over_a_opt(h):
    return np.sqrt(2 * R * h - h**2) / (R - h)

# ---- self-checks (計画書 Test A–E) ----
w0 = wheel_pos(0.0)
assert np.allclose(w0, [-a, -b]), "Test A"
w1 = wheel_pos(np.radians(5))
assert w1[0] < w0[0] and w1[1] > w0[1], "Test B: tiny bump -> rear+up"
eps = 1e-6
slope = (wheel_pos(eps)[0] - w0[0]) / (wheel_pos(eps)[1] - w0[1])
assert np.isclose(-slope, b / a, rtol=1e-3), "Test C: |dx/dz| -> b/a"
# Test D: b/a = s/(R-h) なら初期 Cq ≈ 1
s = np.sqrt(2 * R * H - H**2)
a_d, b_d = a, a * b_over_a_opt(H)
t_d = np.array([-b_d, a_d]) / np.hypot(a_d, b_d)
n_d = np.array([-s, R - H]) / R
assert np.isclose(np.dot(t_d, n_d), 1.0, atol=1e-9), "Test D"
print("self-checks passed (Test A/B/C/D)")

# ---- kinematics table ----
al = np.linspace(0, alpha_max, 200)
W = np.array([wheel_pos(x) for x in al])
dz_w = W[:, 1] - W[0, 1]
mr = np.array([MR(x) for x in al])
ell = np.array([shock_len(x) for x in al])
# wheel force: F_wheel = F_spring * MR, F_spring = k*(ell - ell_free)+preload*k
# shock free length は sag 位置で Fw_static を支えるよう逆算
ell0 = ell[0]
ell_free = ell0 + (Fw_static / (mr[0] * k_spring)) - preload
F_wheel = (k_spring * (ell_free - ell) + preload * k_spring) * mr
k_wheel = np.gradient(np.gradient(F_wheel, dz_w), dz_w) * dz_w + np.gradient(F_wheel, dz_w)
# ponytail: wheel rate は数値微分 1 回で十分
k_wheel = np.gradient(F_wheel, dz_w)

# ---- print summary ----
s = np.sqrt(2 * R * H - H**2)
print(f"\nR={R} h={H}  b/a={b/a:.3f}  推奨 b/a={b_over_a_opt(H):.3f}")
print(f"初期 C_q = {Cq(H):.3f}  (1 に近いほど反力がサス DOF へ効く)")
print(f"wheel travel: {dz_w[-1]:.1f} mm up, {-(W[-1,0]-W[0,0]):.1f} mm rearward")
print(f"shock length: {ell0:.1f} -> {ell[-1]:.1f} mm")
print(f"MR: {mr[0]:.3f} -> {mr[-1]:.3f}")
print(f"wheel rate @static: {k_wheel[0]:.2f} N/mm")

# ---- plots ----
fig, ax = plt.subplots(2, 2, figsize=(11, 8))
ax[0, 0].plot(W[:, 0], W[:, 1], "-o", ms=2)
ax[0, 0].plot(0, 0, "ks", label="pivot")
ax[0, 0].set_aspect("equal")
ax[0, 0].set_title("wheel center path (chassis frame)")
ax[0, 0].set_xlabel("x [mm]"); ax[0, 0].set_ylabel("z [mm]"); ax[0, 0].legend()
ax[0, 0].grid(True)

ax[0, 1].plot(np.degrees(al), mr)
ax[0, 1].set_title("motion ratio vs arm angle")
ax[0, 1].set_xlabel("alpha [deg]"); ax[0, 1].grid(True)

ax[1, 0].plot(dz_w, F_wheel)
ax[1, 0].set_title("wheel force vs wheel travel")
ax[1, 0].set_xlabel("wheel travel [mm]"); ax[1, 0].set_ylabel("N"); ax[1, 0].grid(True)

ax[1, 1].plot(dz_w, k_wheel)
ax[1, 1].set_title("wheel rate vs wheel travel")
ax[1, 1].set_xlabel("wheel travel [mm]"); ax[1, 1].set_ylabel("N/mm"); ax[1, 1].grid(True)

fig.tight_layout()
fig.savefig("outputs_suspension.png", dpi=120)
print("\nsaved outputs_suspension.png")


# ---- GIF animation: 車両が前進して前輪が段差を乗り越える (地面固定視点) ----
from PIL import Image as PILImage

C = np.array([0.0, H])   # 段差角
z_p = R + b              # pivot 高さ (静止時 wheel 中心高 R + b 上)

def arm_vec(al):
    return np.array([-a * np.cos(al) - b * np.sin(al),
                     a * np.sin(al) - b * np.cos(al)])

def solve_alpha(wheel_g):
    """pivot 高 z_p 固定、wheel center = pivot + r(α) を満たす α を 2 分法で解く"""
    lo, hi = -1e-9, alpha_max
    for _ in range(60):
        mid = (lo + hi) / 2
        if wheel_g[1] - z_p > arm_vec(mid)[1]:
            lo = mid
        else:
            hi = mid
    al = (lo + hi) / 2
    return al, np.array([wheel_g[0] - arm_vec(al)[0], z_p])

frames_data = []
# 相1: 地面走行 (α=0, wheel 中心高 R)
for px in np.linspace(-500, C[0] - np.sqrt(2 * R * H - H**2), 25):
    pivot = np.array([px, z_p])
    frames_data.append((pivot, pivot + np.array([-a, -b]), 0.0))
# 相2: 段差角 C に接触。wheel center = C + R*n(φ), φ: 接触法線角
# 地面+段差の両方に接する瞬間 φ0 から、直立 (φ=π/2) まで
phi0 = np.arcsin((R - H) / R)   # = 地面接触が切れる瞬間
for phi in np.linspace(phi0, np.pi / 2, 40):
    wc = C + R * np.array([-np.cos(phi), np.sin(phi)])
    al_mid, pivot = solve_alpha(wc)
    frames_data.append((pivot, wc, al_mid))
# 相3: 段差上走行 — ばね・ダンパのある 1-DOF 振動系で過渡応答を計算
# 車体が段差上に乗った瞬間、ばねは圧縮されたまま → リバウンドして振動 → 収束
# モデル: alpha に関する 1-DOF (準静的接触は終了、wheel は段差上を転がる)
# I*alpha_ddot = -k_t*(alpha - alpha_static) - c_t*alpha_dot
# k_t: アーム角に対する等価トーション剛性 = k_wheel * (dz/dalpha)^2 近似
# alpha_static: 段差上での新しい静的釣り合い角 (sag が減るので小さくなる)

I_arm = m_corner * (L := np.hypot(a, b))**2 * 0.5  # ponytail: 粗い慣性モーメント近似

# 静的釣り合い: F_wheel(alpha) = Fw_static_new (段差上は荷重変わらないと仮定 → 同じ)
# sag 相当角: 数値解で alpha_static を求める

F_wheel_of = lambda al: (k_spring * (ell_free - shock_len(al)) + preload * k_spring) * MR(al)
# 2分法で alpha_static を解く。F(al) は al=0 で最小なので、F(0)>=Fw_static なら sag なし
if F_wheel_of(0.0) >= Fw_static:
    alpha_static = 0.0
else:
    _lo, _hi = 1e-5, alpha_max
    for _ in range(80):
        _mid = (_lo + _hi) / 2
        if F_wheel_of(_mid) < Fw_static:
            _lo = _mid
        else:
            _hi = _mid
    alpha_static = (_lo + _hi) / 2
# 等価トーション剛性 (数値微分)
kt = F_wheel_of(alpha_static + 1e-3) * (a * np.cos(alpha_static + 1e-3) + b * np.sin(alpha_static + 1e-3)) - \
     F_wheel_of(alpha_static - 1e-3) * (a * np.cos(alpha_static - 1e-3) + b * np.sin(alpha_static - 1e-3))
kt /= 2e-3
# 単位を SI に統一: kt [N·mm/rad] → N·m/rad, I [kg·mm²] → kg·m²
kt_SI = kt * 1e-3
I_SI = I_arm * 1e-6
c_t = 0.25 * 2 * np.sqrt(kt_SI * I_SI)   # 減衰比 0.25
zeta = c_t / (2 * np.sqrt(kt_SI * I_SI))
alpha_min = np.radians(-15)              # droop 限界 (shock 最大長相当, TBD)
print(f"\nalpha_static={np.degrees(alpha_static):.2f} deg, kt={kt_SI:.2f} Nm/rad, "
      f"I={I_SI:.4f} kgm2, zeta={zeta:.2f}, f={np.sqrt(kt_SI/I_SI)/2/np.pi:.2f} Hz")

# 相3 開始: 直立通過直後、alpha = alpha_contact_max から開始し alpha_static へ収束
alpha_c_end = al_mid   # 相2 最終の alpha
dt = 0.002
t_sim = 2.0
n_steps = int(t_sim / dt)
alpha_t = np.zeros(n_steps)
alpha_t[0] = alpha_c_end
vel = 0.0
for i in range(1, n_steps):
    torque = -kt_SI * (alpha_t[i-1] - alpha_static) - c_t * vel
    vel += torque / I_SI * dt
    alpha_t[i] = alpha_t[i-1] + vel * dt
    if alpha_t[i] < alpha_min:   # droop 限界
        alpha_t[i] = alpha_min
        vel = max(0.0, vel)

# 相4: 段差上走行 + 過渡応答。wheel は段差面 z=H+R に接触したまま、
# alpha の変化は車体(pivot)高の変化として現れる
wx_start = 0.0
v_chassis = 150.0   # mm/s (表示用)
for i in range(0, n_steps, max(1, n_steps // 30)):
    al_i = alpha_t[i]
    wc = np.array([wx_start + i * dt * v_chassis, H + R])
    frames_data.append((wc - arm_vec(al_i), wc, al_i))

imgs = []
for pivot, wc, al_mid in frames_data:
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot([-700, 0], [0, 0], "k-", lw=2)
    ax.plot([0, 0], [0, H], "k-", lw=2)
    ax.plot([0, 700], [H, H], "k-", lw=2)
    ax.plot([pivot[0], wc[0]], [pivot[1], wc[1]], "b-", lw=3)
    ax.plot(*pivot, "ks", ms=8, label="pivot")
    th = np.linspace(0, 2 * np.pi, 60)
    ax.plot(wc[0] + R * np.cos(th), wc[1] + R * np.sin(th), "g-", lw=2)
    ax.plot(*wc, "g.", ms=6)
    spoke = np.array([np.cos(-al_mid), np.sin(-al_mid)])
    ax.plot([wc[0] - R * spoke[0], wc[0] + R * spoke[0]],
            [wc[1] - R * spoke[1], wc[1] + R * spoke[1]], "g--", lw=1)
    S = pivot + rho * (wc - pivot)
    U_g = pivot + np.array(U)
    ax.plot([S[0], U_g[0]], [S[1], U_g[1]], "r-", lw=4, alpha=0.6, label="shock")
    ax.set_xlim(-650, 650); ax.set_ylim(-40, 320)
    ax.set_aspect("equal")
    ax.set_title(f"front trailing arm over {H:.0f}mm step   alpha={np.degrees(al_mid):.1f}deg")
    ax.legend(loc="upper left", fontsize=8)
    fig.canvas.draw()
    img = np.asarray(fig.canvas.buffer_rgba())[..., :3]
    imgs.append(PILImage.fromarray(img))
    plt.close(fig)

imgs[0].save("step_climb.gif", save_all=True, append_images=imgs[1:],
             duration=60, loop=0)
print("saved step_climb.gif")
