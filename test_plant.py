import matplotlib.pyplot as plt
import numpy as np
from plant import Plant1st, Plant2nd

def ku_for_target_vmax(c0, c2, vmax_mps, u_ss):
    return (c0 + c2 * (vmax_mps ** 2)) / max(u_ss, 1e-6)

def run_plant1st():
    plant = Plant1st()
    plant.reset(0.0)

    N = 200  
    v_traj, u_traj, t_traj = [], [], []
    for k in range(N):
        u_cmd = 0.8
        state, y = plant.step(u_cmd)
        v_traj.append(y)
        u_traj.append(plant.u_prev)
        t_traj.append(k * plant.Ts)

    plt.figure()
    plt.plot(t_traj, v_traj, label="Velocity (m/s)")
    plt.plot(t_traj, u_traj, label="Input u")
    plt.xlabel("Time (s)"); plt.ylabel("Value"); plt.legend()
    plt.title("Plant1st Step Test"); plt.grid(True)
    plt.show()

def run_plant2nd_racecar():

    Ts = 0.02
    m = 1200.0
    Crr = 0.012
    c0 = Crr * m * 9.81            
    rho = 1.225; CdA = 0.6
    c2 = 0.5 * rho * CdA            
    tau_a = 0.20

    vmax_kmh = 200.0
    vmax_mps = vmax_kmh / 3.6

    u_ss = 0.70

    ku = ku_for_target_vmax(c0, c2, vmax_mps, u_ss)

    plant = Plant2nd(m=m, ku=ku, c0=c0, c1=0.0, c2=c2, tau_a=tau_a, Ts=Ts,
                     umin=0.0, umax=1.0, du_max=0.05)
    plant.reset(0.0)

    T = 30.0
    N = int(T / Ts)
    t = np.arange(N) * Ts
    v = np.zeros(N); u_hist = np.zeros(N)

    # 油門：先 S-curve 1 秒（減少 jerk），再維持 u_ss
    def throttle_profile(k):
        trise = int(1.0 / Ts)
        x = min(1.0, k / trise)
        s = x*x*(3 - 2*x)  # smoothstep
        return u_ss * s

    for k in range(N):
        u_cmd = throttle_profile(k)
        state, y = plant.step(u_cmd)
        v[k] = y
        u_hist[k] = plant.u_prev

    kmh = v * 3.6
    v_ss = kmh[-int(2.0/Ts):].mean()

    fig, ax = plt.subplots(2,1, figsize=(8,6), sharex=True)
    ax[0].plot(t, kmh, lw=1.6)
    ax[0].grid(True); ax[0].set_ylabel("Speed (km/h)")
    ax[0].set_title(f"Racecar Longitudinal Model — target ≈ {vmax_kmh:.0f} km/h (u≈{u_ss:.2f})")
    ax[0].axhline(vmax_kmh, color='k', ls='--', lw=1, label="Desired vmax")
    ax[0].legend()

    ax[1].plot(t, u_hist, lw=1.4)
    ax[1].grid(True); ax[1].set_ylabel("Throttle u"); ax[1].set_xlabel("Time (s)")

    plt.tight_layout(); plt.show()
    print(f"Approx steady speed: {v_ss:.1f} km/h")

if __name__ == "__main__":
    run_plant1st()
    run_plant2nd_racecar()
