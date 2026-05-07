import socket
import json
import threading

import tkinter as tk
from tkinter import ttk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

# =========================================================
# CONEXIÓN TCP
# =========================================================

HOST = "192.168.4.1"
PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

print("Conectando...")
sock.connect((HOST, PORT))
print("Conectado al ESP32")

# =========================================================
# VARIABLES
# =========================================================

x_vals = []
y_vals = []
wr_vals     = []
wrref_vals  = []
tiempo_vals = []
t_inicio    = None
import time as _time

buffer = ""

# =========================================================
# VENTANA TKINTER
# =========================================================

root = tk.Tk()
root.title("Control Robot ESP32")
root.geometry("1000x700")

# =========================================================
# FUNCIONES ENVÍO
# =========================================================

def enviar(cmd):
    try:
        mensaje = cmd + "\n"
        sock.sendall(mensaje.encode())
        print("ENVIADO:", cmd)

    except Exception as e:
        print("ERROR ENVIANDO:", e)

# =========================================================
# BOTONES
# =========================================================

def start_robot():
    enviar("START")

def stop_robot():
    enviar("STOP")
    
def reset_grafica():
    global t_inicio
    x_vals.clear()
    y_vals.clear()
    wr_vals.clear()
    wrref_vals.clear()
    tiempo_vals.clear()
    t_inicio = None
    print("Gráfica reseteada")

# =========================================================
# ACTUALIZAR PARÁMETROS
# =========================================================

def actualizar_tx(valor):
    enviar(f"TX:{valor}")

def actualizar_ty(valor):
    enviar(f"TY:{valor}")

def actualizar_kv(valor):
    enviar(f"KV:{valor}")

def actualizar_kw(valor):
    enviar(f"KW:{valor}")

def actualizar_vm(valor):
    enviar(f"VM:{valor}")

def actualizar_wm(valor):
    enviar(f"WM:{valor}")
    
def actualizar_or(valor): enviar(f"OR:{valor}")
def actualizar_ol(valor): enviar(f"OL:{valor}")

# =========================================================
# PANEL CONTROLES
# =========================================================

panel = tk.Frame(root)
panel.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

# START STOP

btn_start = tk.Button(panel, text="START", command=start_robot, bg="green")
btn_start.pack(fill="x")

btn_stop = tk.Button(panel, text="STOP", command=stop_robot, bg="red")
btn_stop.pack(fill="x", pady=5)

btn_reset = tk.Button(panel, text="RESET GRÁFICA", command=reset_grafica, bg="blue", fg="white")
btn_reset.pack(fill="x", pady=5)

# =========================================================
# SLIDERS
# =========================================================

# TARGET X
tk.Label(panel, text="Target X").pack()

slider_tx = tk.Scale(
    panel,
    from_=-100,
    to=100,
    resolution=1,
    orient=tk.HORIZONTAL,
    command=actualizar_tx
)
slider_tx.pack(fill="x")

# TARGET Y
tk.Label(panel, text="Target Y").pack()

slider_ty = tk.Scale(
    panel,
    from_=-100,
    to=200,
    resolution=1,
    orient=tk.HORIZONTAL,
    command=actualizar_ty
)
slider_ty.set(100)
slider_ty.pack(fill="x")

# Kv
tk.Label(panel, text="Kv").pack()

slider_kv = tk.Scale(
    panel,
    from_=0,
    to=2,
    resolution=0.01,
    orient=tk.HORIZONTAL,
    command=actualizar_kv
)
slider_kv.set(0.4)
slider_kv.pack(fill="x")

# Kw
tk.Label(panel, text="Kw").pack()

slider_kw = tk.Scale(
    panel,
    from_=0,
    to=2,
    resolution=0.01,
    orient=tk.HORIZONTAL,
    command=actualizar_kw
)
slider_kw.set(0.6)
slider_kw.pack(fill="x")

# V_MAX
tk.Label(panel, text="V_MAX").pack()

slider_vm = tk.Scale(
    panel,
    from_=0,
    to=5,
    resolution=0.1,
    orient=tk.HORIZONTAL,
    command=actualizar_vm
)
slider_vm.set(1.5)
slider_vm.pack(fill="x")

# W_MAX
tk.Label(panel, text="W_MAX").pack()

slider_wm = tk.Scale(
    panel,
    from_=0,
    to=5,
    resolution=0.1,
    orient=tk.HORIZONTAL,
    command=actualizar_wm
)
slider_wm.set(1.5)
slider_wm.pack(fill="x")

ttk.Separator(panel, orient="horizontal").pack(fill="x", pady=8)
tk.Label(panel, text="── Calibración Offsets ──", fg="gray").pack()

tk.Label(panel, text="Offset R").pack()
slider_or = tk.Scale(panel, from_=50, to=150, resolution=1, orient=tk.HORIZONTAL, command=actualizar_or)
slider_or.set(90)
slider_or.pack(fill="x")

tk.Label(panel, text="Offset L").pack()
slider_ol = tk.Scale(panel, from_=50, to=150, resolution=1, orient=tk.HORIZONTAL, command=actualizar_ol)
slider_ol.set(90)
slider_ol.pack(fill="x")

# =========================================================
# MATPLOTLIB
# =========================================================

fig, (ax, ax_vel) = plt.subplots(2, 1, figsize=(6, 7), gridspec_kw={"height_ratios": [1.2, 1]})

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

# =========================================================
# RECEPCIÓN DATOS
# =========================================================

def recibir_datos():

    global buffer, t_inicio

    while True:

        try:

            data = sock.recv(1024).decode()

            if not data:
                continue

            buffer += data

            while "\n" in buffer:

                line, buffer = buffer.split("\n", 1)

                line = line.strip()

                if not line:
                    continue

                try:

                    obj = json.loads(line)

                    x = obj["x"]
                    y = obj["y"]

                    print(f"X={x}  Y={y}")

                    x_vals.append(x)
                    y_vals.append(y)
                    ahora = _time.time()
                    if t_inicio is None:
                        t_inicio = ahora
                    tiempo_vals.append(ahora - t_inicio)
                    wr_vals.append(obj.get("wr", 0))
                    wrref_vals.append(obj.get("wrref", 0))

                except Exception as e:

                    print("JSON ERROR:", e)
                    print(line)

        except Exception as e:

            print("ERROR SOCKET:", e)

# =========================================================
# ANIMACIÓN
# =========================================================

def update(frame):

    ax.clear()

    ax.plot(x_vals, y_vals)

    ax.set_title("Trayectoria Robot")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    ax.grid(True)

    ax.axis("equal")
    ax_vel.clear()
    t_plot   = tiempo_vals[-300:]
    wr_plot  = wr_vals[-300:]
    ref_plot = wrref_vals[-300:]
    if t_plot:
        ax_vel.plot(t_plot, ref_plot, color="orange", linestyle="--", linewidth=1.2, label="ωR ref")
        ax_vel.plot(t_plot, wr_plot,  color="green",  linewidth=1.2, label="ωR real")
    ax_vel.set_title("Velocidad rueda derecha")
    ax_vel.set_xlabel("Tiempo (s)")
    ax_vel.set_ylabel("ω (rad/s)")
    ax_vel.legend(loc="upper left", fontsize=8)
    ax_vel.grid(True)
    fig.tight_layout(pad=2.5)

ani = FuncAnimation(
    fig,
    update,
    interval=100,
    cache_frame_data=False
)

# =========================================================
# HILO RECEPCIÓN
# =========================================================

thread = threading.Thread(target=recibir_datos)
thread.daemon = True
thread.start()

# =========================================================
# MAIN LOOP
# =========================================================

root.mainloop()