"""
Simulação cinemática 3D de um quadricóptero.
Uso:
  python src/main.py
  python src/main.py --plots-only
  python src/main.py --save-gif   # requer Pillow
"""
from pathlib import Path
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# Parâmetros ilustrativos da animação (não são dados de um drone real)
DURATION = 20.0       # segundos
FPS = 30
N_POINTS = 600
RADIUS = 4.0          # escala horizontal em metros
Z_BASE = 3.0          # altitude média em metros
Z_AMPLITUDE = 1.0     # amplitude vertical em metros
DRONE_ARM = 0.45      # tamanho visual dos braços (m)
PROP_RADIUS = 0.18    # tamanho visual das hélices (m)

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs"
OUT.mkdir(exist_ok=True)


def rotation_matrix(roll, pitch, yaw):
    """Matriz R_GB = Rz(yaw) @ Ry(pitch) @ Rx(roll). Ângulos em radianos."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def make_trajectory():
    t = np.linspace(0.0, DURATION, N_POINTS)
    omega = 2 * np.pi / DURATION
    x = RADIUS * np.sin(omega * t)
    y = (RADIUS / 2) * np.sin(2 * omega * t)
    z = Z_BASE + Z_AMPLITUDE * np.sin(omega * t / 2)
    pos = np.column_stack((x, y, z))
    vel = np.gradient(pos, t, axis=0, edge_order=2)
    vx, vy, vz = vel.T
    horizontal_speed = np.sqrt(vx**2 + vy**2)
    yaw = np.unwrap(np.arctan2(vy, vx))
    pitch = np.arctan2(-vz, np.maximum(horizontal_speed, 1e-8))
    roll = np.zeros_like(t)
    euler = np.column_stack((roll, pitch, yaw))
    speed = np.linalg.norm(vel, axis=1)
    return t, pos, vel, speed, euler


def plot_results(t, pos, vel, speed, euler):
    # Trajetória 3D
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], linewidth=2, label="Trajetória")
    ax.scatter(*pos[0], marker="o", s=55, label="Início")
    ax.scatter(*pos[-1], marker="X", s=65, label="Fim")
    ax.set_title("Trajetória 3D do drone")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    ax.set_box_aspect((1, 1, 0.65))
    fig.tight_layout()
    fig.savefig(OUT / "posicao_3d.png", dpi=160)
    plt.close(fig)

    # Posição por tempo
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(t, pos[:, 0], label="X")
    ax.plot(t, pos[:, 1], label="Y")
    ax.plot(t, pos[:, 2], label="Z")
    ax.set_title("Posição do drone ao longo do tempo")
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Posição (m)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "posicoes_tempo.png", dpi=160)
    plt.close(fig)

    # Velocidade por tempo
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(t, vel[:, 0], label="Vx")
    ax.plot(t, vel[:, 1], label="Vy")
    ax.plot(t, vel[:, 2], label="Vz")
    ax.plot(t, speed, linewidth=2.2, label="Módulo da velocidade")
    ax.set_title("Velocidade do drone ao longo do tempo")
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Velocidade (m/s)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(OUT / "velocidade_tempo.png", dpi=160)
    plt.close(fig)

    # Orientação
    fig, ax = plt.subplots(figsize=(10, 5.5))
    labels = ["Roll (φ)", "Pitch (θ)", "Yaw (ψ)"]
    for i, label in enumerate(labels):
        ax.plot(t, np.rad2deg(euler[:, i]), label=label)
    ax.set_title("Orientação estimada a partir da trajetória")
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Ângulo (graus)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "orientacao_tempo.png", dpi=160)
    plt.close(fig)


def local_to_global(points_local, position, rotation):
    """Transforma pontos (N,3) do referencial do drone para o global."""
    return (rotation @ points_local.T).T + position


def make_animation(t, pos, euler, save_gif=False):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], linestyle="--",
            linewidth=1.2, alpha=0.45, label="Trajetória planejada")
    trail, = ax.plot([], [], [], linewidth=2, label="Rastro percorrido")
    body_lines = [ax.plot([], [], [], linewidth=4)[0] for _ in range(2)]
    props = [ax.plot([], [], [], linewidth=2)[0] for _ in range(4)]
    axes_lines = [ax.plot([], [], [], linewidth=2)[0] for _ in range(3)]
    drone_dot, = ax.plot([], [], [], marker="o", markersize=5)
    status = ax.text2D(0.03, 0.95, "", transform=ax.transAxes)

    ax.set_title("Simulação 3D — drone seguindo trajetória")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    pad = 1.2
    ax.set_xlim(pos[:, 0].min()-pad, pos[:, 0].max()+pad)
    ax.set_ylim(pos[:, 1].min()-pad, pos[:, 1].max()+pad)
    ax.set_zlim(0, pos[:, 2].max()+2)
    ax.set_box_aspect((1, 1, 0.65))
    ax.legend(loc="upper right")

    # Geometria local: dois braços cruzados no plano XY do drone
    arm = DRONE_ARM
    arms_local = [
        np.array([[-arm, 0, 0], [arm, 0, 0]]),
        np.array([[0, -arm, 0], [0, arm, 0]])
    ]
    rotor_local = np.array([
        [ arm, 0, 0], [-arm, 0, 0], [0, arm, 0], [0, -arm, 0]
    ])
    rotor_circle = np.linspace(0, 2*np.pi, 32)
    circle_local = np.column_stack((
        PROP_RADIUS*np.cos(rotor_circle),
        PROP_RADIUS*np.sin(rotor_circle),
        np.zeros_like(rotor_circle)
    ))

    stride = max(1, len(t)//180)
    frames = list(range(0, len(t), stride))
    if frames[-1] != len(t)-1:
        frames.append(len(t)-1)

    def update(idx):
        p = pos[idx]
        roll, pitch, yaw = euler[idx]
        R = rotation_matrix(roll, pitch, yaw)
        trail.set_data(pos[:idx+1, 0], pos[:idx+1, 1])
        trail.set_3d_properties(pos[:idx+1, 2])
        for i, arm_pts in enumerate(arms_local):
            pts = local_to_global(arm_pts, p, R)
            body_lines[i].set_data(pts[:, 0], pts[:, 1])
            body_lines[i].set_3d_properties(pts[:, 2])
        for i, center_local in enumerate(rotor_local):
            circle_pts = circle_local + center_local
            pts = local_to_global(circle_pts, p, R)
            props[i].set_data(pts[:, 0], pts[:, 1])
            props[i].set_3d_properties(pts[:, 2])
        # Eixos locais x, y, z, desenhados a partir do centro do drone
        axis_len = 0.75
        unit_axes = np.eye(3) * axis_len
        for i in range(3):
            endpoints = local_to_global(np.array([[0,0,0], unit_axes[i]]), p, R)
            axes_lines[i].set_data(endpoints[:, 0], endpoints[:, 1])
            axes_lines[i].set_3d_properties(endpoints[:, 2])
        drone_dot.set_data([p[0]], [p[1]])
        drone_dot.set_3d_properties([p[2]])
        status.set_text(
            f"t = {t[idx]:.1f} s | X={p[0]:.2f} m | Y={p[1]:.2f} m | Z={p[2]:.2f} m\n"
            f"roll={np.rad2deg(roll):.1f}°  pitch={np.rad2deg(pitch):.1f}°  yaw={np.rad2deg(yaw):.1f}°"
        )
        return [trail, *body_lines, *props, *axes_lines, drone_dot, status]

    ani = FuncAnimation(fig, update, frames=frames, interval=1000/FPS,
                        blit=False, repeat=True)
    fig.tight_layout()
    if save_gif:
        gif_path = OUT / "animacao_drone.gif"
        ani.save(gif_path, writer=PillowWriter(fps=FPS))
        print(f"GIF salvo em: {gif_path}")
    else:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Simulação 3D cinemática de drone")
    parser.add_argument("--plots-only", action="store_true", help="gera apenas os gráficos")
    parser.add_argument("--save-gif", action="store_true", help="salva a animação como GIF (requer Pillow)")
    args = parser.parse_args()

    t, pos, vel, speed, euler = make_trajectory()
    plot_results(t, pos, vel, speed, euler)
    print("Gráficos salvos em:", OUT)
    print(f"Altitude: {pos[:,2].min():.2f} a {pos[:,2].max():.2f} m")
    print(f"Velocidade máxima estimada: {speed.max():.2f} m/s")
    if not args.plots_only:
        make_animation(t, pos, euler, save_gif=args.save_gif)


if __name__ == "__main__":
    main()
