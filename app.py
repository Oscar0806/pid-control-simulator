import gradio as gr
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Required for non-interactive backend
import matplotlib.pyplot as plt
from pid_engine import simulate_pid, ziegler_nichols_tuning
 
def run_simulation(Kp, Ki, Kd, plant_type, plant_gain,
                    plant_tau, plant_wn, plant_zeta,
                    setpoint, t_end, dist_time, dist_mag):
    """Main simulation function – called by Gradio when user clicks Run."""
    
    d_time = dist_time if dist_time > 0 else None
    
    r = simulate_pid(
        Kp=Kp, Ki=Ki, Kd=Kd,
        plant_type=plant_type, plant_gain=plant_gain,
        plant_tau=plant_tau, plant_wn=plant_wn,
        plant_zeta=plant_zeta, setpoint=setpoint,
        t_end=t_end, disturbance_time=d_time,
        disturbance_mag=dist_mag
    )
    
    # ── PLOT 1: Step Response ──
    fig1, ax1 = plt.subplots(figsize=(10, 4))
    ax1.plot(r["t"], r["y"], "b-", linewidth=2, label="Output y(t)")
    ax1.plot(r["t"], r["sp"], "r--", linewidth=1, label="Setpoint")
    ax1.axhline(y=setpoint*1.02, color="gray", linestyle=":",
                alpha=0.5, label="±2% band")
    ax1.axhline(y=setpoint*0.98, color="gray", linestyle=":", alpha=0.5)
    ax1.fill_between(r["t"], setpoint*0.98, setpoint*1.02,
                      alpha=0.05, color="green")
    ax1.set_xlabel("Time (s)", fontsize=12)
    ax1.set_ylabel("Output", fontsize=12)
    ax1.set_title("Closed-Loop Step Response", fontsize=14, fontweight="bold")
    ax1.legend(loc="lower right")
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(0, t_end)
    plt.tight_layout()
    
    # ── PLOT 2: Control Signal ──
    fig2, ax2 = plt.subplots(figsize=(10, 3))
    ax2.plot(r["t"], r["u"], "g-", linewidth=1.5, label="u(t)")
    ax2.set_xlabel("Time (s)", fontsize=12)
    ax2.set_ylabel("Control signal u(t)", fontsize=12)
    ax2.set_title("Controller Output", fontsize=14, fontweight="bold")
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_xlim(0, t_end)
    plt.tight_layout()
    
    # ── PLOT 3: PID Components ──
    fig3, ax3 = plt.subplots(figsize=(10, 3))
    ax3.plot(r["t"], r["P"], label="P (proportional)", alpha=0.8)
    ax3.plot(r["t"], r["I"], label="I (integral)", alpha=0.8)
    ax3.plot(r["t"], r["D"], label="D (derivative)", alpha=0.8)
    ax3.set_xlabel("Time (s)", fontsize=12)
    ax3.set_ylabel("Component value", fontsize=12)
    ax3.set_title("PID Component Breakdown", fontsize=14, fontweight="bold")
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(0, t_end)
    plt.tight_layout()
    
    # ── PLOT 4: Error ──
    fig4, ax4 = plt.subplots(figsize=(10, 3))
    ax4.fill_between(r["t"], r["e"], alpha=0.3, color="red")
    ax4.plot(r["t"], r["e"], "r-", linewidth=1, label="Error e(t)")
    ax4.axhline(y=0, color="black", linewidth=0.5)
    ax4.set_xlabel("Time (s)", fontsize=12)
    ax4.set_ylabel("Error", fontsize=12)
    ax4.set_title(f"Tracking Error | IAE = {r['iae']:.3f}", fontsize=14, fontweight="bold")
    ax4.grid(True, alpha=0.3)
    ax4.set_xlim(0, t_end)
    plt.tight_layout()
    
    # ── Performance Metrics Text ──
    metrics = "═══════════════════════════════════════\n"
    metrics += "         PERFORMANCE METRICS\n"
    metrics += "═══════════════════════════════════════\n"
    metrics += f"Rise Time (10%→90%):  {r['rise_time']} s\n"
    metrics += f"Overshoot:            {r['overshoot_pct']}%\n"
    metrics += f"Settling Time (±2%):  {r['settling_time']} s\n"
    metrics += f"Steady-State Error:   {r['ss_error']}\n"
    metrics += f"IAE:                  {r['iae']}\n"
    metrics += "\n───── Ziegler-Nichols Tuning (Ku=4, Tu=2) ─────\n"
    zn = ziegler_nichols_tuning(Ku=4.0, Tu=2.0)
    for name, params in zn.items():
        metrics += f"  {name:8s}: Kp={params['Kp']}, Ki={params['Ki']}, Kd={params['Kd']}\n"
    metrics += "\n───── Interpretation ─────\n"
    if r['overshoot_pct'] > 20:
        metrics += "⚠ High overshoot: reduce Kp or increase Kd\n"
    if r['ss_error'] > 0.01:
        metrics += "⚠ Steady-state error: increase Ki\n"
    if r['settling_time'] > 10:
        metrics += "⚠ Slow settling: increase Kp or Ki\n"
    if r['overshoot_pct'] < 5 and r['ss_error'] < 0.01 and r['settling_time'] < 5:
        metrics += "✅ Well-tuned! Good balance of speed and stability\n"
    
    return fig1, fig2, fig3, fig4, metrics
 
# ══════════════════════════════════
#  GRADIO INTERFACE DEFINITION
# ══════════════════════════════════
with gr.Blocks(title="PID Control Simulator",
               theme=gr.themes.Soft()) as demo:
    
    gr.Markdown("# 🎯 PID Control System Simulator & Tuner")
    gr.Markdown("**Interactive simulation of closed-loop PID control "
                "for first-order and second-order plants. "
                "Modelling & Simulation + Smart Automation concept**"
                )
    gr.Markdown("---")
    
    with gr.Row():
        with gr.Column(scale=1):
            gr.Markdown("### 🎛️ PID Controller Gains")
            kp = gr.Slider(0, 20, value=2.0, step=0.1,
                           label="Kp (Proportional gain)")
            ki = gr.Slider(0, 20, value=1.0, step=0.1,
                           label="Ki (Integral gain)")
            kd = gr.Slider(0, 10, value=0.5, step=0.1,
                           label="Kd (Derivative gain)")
            
            gr.Markdown("### 🏭 Plant Model")
            plant = gr.Dropdown(
                ["first_order", "second_order"],
                value="second_order",
                label="Plant type")
            gain = gr.Slider(0.1, 5, value=1.0, step=0.1,
                             label="Plant gain K")
            tau = gr.Slider(0.1, 10, value=1.0, step=0.1,
                            label="Time constant τ (1st order only)")
            wn = gr.Slider(0.1, 10, value=1.0, step=0.1,
                           label="Natural frequency ωn (2nd order)")
            zeta = gr.Slider(0, 2, value=0.5, step=0.05,
                             label="Damping ratio ζ (2nd order)")
            
            gr.Markdown("### ⚙️ Simulation Settings")
            sp_val = gr.Slider(0.1, 5, value=1.0, step=0.1,
                               label="Setpoint (desired value)")
            t_end = gr.Slider(5, 50, value=20, step=1,
                              label="Simulation duration (seconds)")
            d_time = gr.Slider(0, 30, value=0, step=1,
                               label="Disturbance at time (s), 0=none")
            d_mag = gr.Slider(-2, 2, value=0.3, step=0.1,
                              label="Disturbance magnitude")
            
            btn = gr.Button("▶ Run Simulation", variant="primary")
        
        with gr.Column(scale=2):
            plot1 = gr.Plot(label="Step Response")
            plot2 = gr.Plot(label="Control Signal u(t)")
            plot3 = gr.Plot(label="PID Components (P, I, D)")
            plot4 = gr.Plot(label="Error e(t)")
            metrics = gr.Textbox(label="Performance Metrics & Tuning",
                                 lines=16)
    
    btn.click(
        fn=run_simulation,
        inputs=[kp, ki, kd, plant, gain, tau, wn, zeta,
                sp_val, t_end, d_time, d_mag],
        outputs=[plot1, plot2, plot3, plot4, metrics]
    )
    
    gr.Markdown("---")
    gr.Markdown("*PID Control System Simulator | Built by Oscar Vincent Dbritto*"
                )
 
if __name__ == "__main__":
    demo.launch()
