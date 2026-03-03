def plot_results(node):

    dx = integrate_displacement(node.t, node.ax)
    dy = integrate_displacement(node.t, node.ay)
    dz = integrate_displacement(node.t, node.az)

    fx, mx, fs = compute_fft(node.t, node.ax)
    fy, my, _ = compute_fft(node.t, node.ay)
    fz, mz, _ = compute_fft(node.t, node.az)

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))

    # FFT acceleration
    axes[0, 0].plot(fx, mx)
    annotate_top_peaks(axes[0, 0], fx, mx)
    axes[0, 0].set_title("FFT accel X")
    axes[0, 0].set_xlabel("Hz")
    axes[0, 0].grid(True)

    axes[0, 1].plot(fy, my)
    annotate_top_peaks(axes[0, 1], fy, my)
    axes[0, 1].set_title("FFT accel Y")
    axes[0, 1].set_xlabel("Hz")
    axes[0, 1].grid(True)

    axes[0, 2].plot(fz, mz)
    annotate_top_peaks(axes[0, 2], fz, mz)
    axes[0, 2].set_title("FFT accel Z")
    axes[0, 2].set_xlabel("Hz")
    axes[0, 2].grid(True)

    # Displacement orbits
    axes[1, 0].plot(dx, dy)
    axes[1, 0].set_title("Displacement Orbit XY")
    axes[1, 0].axis('equal')
    axes[1, 0].grid(True)

    axes[1, 1].plot(dx, dz)
    axes[1, 1].set_title("Displacement Orbit XZ")
    axes[1, 1].axis('equal')
    axes[1, 1].grid(True)

    axes[1, 2].plot(dy, dz)
    axes[1, 2].set_title("Displacement Orbit YZ")
    axes[1, 2].axis('equal')
    axes[1, 2].grid(True)

    fig.suptitle(
        f"IMU FFT + Displacement Orbits (IMU freq ≈ {fs:.1f} Hz)",
        fontsize=16
    )

    plt.tight_layout()

    # ---- SAVE PDF ----
    pdf_filename = "imu_fft_results.pdf"
    fig.savefig(pdf_filename, format="pdf")
    print(f"Saved figure to {pdf_filename}")

    # ---- GENERATE LATEX FILE ----
    tex_filename = "imu_fft_results.tex"

    latex_content = rf"""
\begin{{figure}}[ht]
    \centering
    \includegraphics[width=\textwidth]{{{pdf_filename}}}
    \caption{{IMU FFT and displacement orbit analysis (Sampling frequency $\approx {fs:.1f}$ Hz).}}
    \label{{fig:imu_fft}}
\end{{figure}}
"""

    with open(tex_filename, "w") as f:
        f.write(latex_content.strip())

    print(f"Generated LaTeX snippet: {tex_filename}")

    plt.show()
