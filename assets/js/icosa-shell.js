// Portfolio icosahedron canvas: shared by the linked project pages.
const ICOSA_VERTICES = (() => {
    const phi = (1 + Math.sqrt(5)) / 2;
    const raw = [
        [-1, phi, 0], [1, phi, 0], [-1, -phi, 0], [1, -phi, 0],
        [0, -1, phi], [0, 1, phi], [0, -1, -phi], [0, 1, -phi],
        [phi, 0, -1], [phi, 0, 1], [-phi, 0, -1], [-phi, 0, 1],
    ];
    return raw.map(([x, y, z]) => {
        const mag = Math.hypot(x, y, z);
        return { x: x / mag, y: y / mag, z: z / mag };
    });
})();

const ICOSA_EDGES = (() => {
    const edges = [];
    for (let i = 0; i < ICOSA_VERTICES.length; i += 1) {
        for (let j = i + 1; j < ICOSA_VERTICES.length; j += 1) {
            const a = ICOSA_VERTICES[i];
            const b = ICOSA_VERTICES[j];
            const dist = Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
            if (dist < 1.12) {
                edges.push([i, j]);
            }
        }
    }
    return edges;
})();

const rotateVertex = (vertex, rx, ry, rz) => {
    let { x, y, z } = vertex;

    const cosX = Math.cos(rx);
    const sinX = Math.sin(rx);
    const y1 = y * cosX - z * sinX;
    const z1 = y * sinX + z * cosX;
    y = y1;
    z = z1;

    const cosY = Math.cos(ry);
    const sinY = Math.sin(ry);
    const x2 = x * cosY + z * sinY;
    const z2 = -x * sinY + z * cosY;
    x = x2;
    z = z2;

    const cosZ = Math.cos(rz);
    const sinZ = Math.sin(rz);
    const x3 = x * cosZ - y * sinZ;
    const y3 = x * sinZ + y * cosZ;

    return { x: x3, y: y3, z };
};

const setupIcosaShell = (canvas) => {
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const isNavShell = !!canvas.closest(".brand-mark");

    let width = 0;
    let height = 0;
    let rotationX = 0.8;
    let rotationY = 0.55;
    let rotationZ = 0.12;
    let velocityX = 0.022;
    let velocityY = 0.048;
    let velocityZ = 0.014;
    let dragging = false;
    let didDrag = false;
    let lastPointerX = 0;
    let lastPointerY = 0;
    let lastPointerTime = 0;

    const resize = () => {
        const rect = canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        width = rect.width;
        height = rect.height;
        canvas.width = Math.max(1, Math.round(rect.width * dpr));
        canvas.height = Math.max(1, Math.round(rect.height * dpr));
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    };

    const draw = () => {
        ctx.clearRect(0, 0, width, height);
        const size = Math.min(width, height);
        const radius = size * (isNavShell ? 0.445 : 0.4);
        const distance = 2.8;
        const projected = ICOSA_VERTICES.map((vertex) => {
            const rotated = rotateVertex(vertex, rotationX, rotationY, rotationZ);
            const scale = radius / (rotated.z + distance);
            return {
                x: width * 0.5 + rotated.x * scale,
                y: height * 0.5 + rotated.y * scale,
                z: rotated.z,
                scale,
            };
        });

        ICOSA_EDGES.forEach(([aIndex, bIndex]) => {
            const a = projected[aIndex];
            const b = projected[bIndex];
            const depth = (a.z + b.z) * 0.5;
            const alpha = 0.26 + (depth + 1) * 0.28;
            const edgeAlpha = Math.max(0.18, Math.min(0.84, alpha));
            ctx.strokeStyle = depth > 0
                ? `rgba(255, 132, 126, ${edgeAlpha})`
                : `rgba(176, 86, 94, ${Math.max(0.12, edgeAlpha * 0.58)})`;
            ctx.lineWidth = depth > 0 ? (isNavShell ? 2.1 : 2.4) : (isNavShell ? 1.15 : 1.4);
            ctx.beginPath();
            ctx.moveTo(a.x, a.y);
            ctx.lineTo(b.x, b.y);
            ctx.stroke();
        });

        projected
            .slice()
            .sort((a, b) => a.z - b.z)
            .forEach((point) => {
                const r = point.z > 0
                    ? (isNavShell ? 2.65 : 2.95)
                    : (isNavShell ? 1.8 : 2.0);
                const orb = ctx.createRadialGradient(point.x - r * 0.35, point.y - r * 0.4, r * 0.15, point.x, point.y, r * 1.45);
                if (point.z > 0) {
                    orb.addColorStop(0, "rgba(255, 249, 233, 0.98)");
                    orb.addColorStop(0.46, "rgba(244, 204, 109, 0.96)");
                    orb.addColorStop(1, "rgba(214, 141, 83, 0.88)");
                } else {
                    orb.addColorStop(0, "rgba(248, 220, 161, 0.76)");
                    orb.addColorStop(1, "rgba(185, 112, 72, 0.56)");
                }
                ctx.fillStyle = orb;
                ctx.beginPath();
                ctx.arc(point.x, point.y, r, 0, Math.PI * 2);
                ctx.fill();

                if (point.z > 0) {
                    ctx.shadowColor = "rgba(244, 202, 116, 0.48)";
                    ctx.shadowBlur = isNavShell ? 8 : 10;
                    ctx.beginPath();
                    ctx.arc(point.x, point.y, r * 0.55, 0, Math.PI * 2);
                    ctx.fillStyle = "rgba(255, 247, 228, 0.78)";
                    ctx.fill();
                    ctx.shadowBlur = 0;
                }
            });

        const glow = ctx.createRadialGradient(width * 0.5, height * 0.5, size * 0.06, width * 0.5, height * 0.5, size * (isNavShell ? 0.6 : 0.56));
        glow.addColorStop(0, "rgba(255, 122, 127, 0.2)");
        glow.addColorStop(1, "rgba(255, 122, 127, 0)");
        ctx.globalCompositeOperation = "lighter";
        ctx.fillStyle = glow;
        ctx.beginPath();
        ctx.arc(width * 0.5, height * 0.5, size * (isNavShell ? 0.6 : 0.56), 0, Math.PI * 2);
        ctx.fill();
        ctx.globalCompositeOperation = "source-over";
    };

    const animate = () => {
        if (!dragging) {
            velocityX *= 0.992;
            velocityY *= 0.992;
            velocityZ *= 0.992;
        }

        rotationX += velocityX;
        rotationY += velocityY;
        rotationZ += velocityZ;
        draw();
        window.requestAnimationFrame(animate);
    };

    const onPointerDown = (event) => {
        event.stopPropagation();
        dragging = true;
        didDrag = false;
        canvas.classList.add("dragging");
        lastPointerX = event.clientX;
        lastPointerY = event.clientY;
        lastPointerTime = performance.now();
        canvas.setPointerCapture(event.pointerId);
    };

    const onPointerMove = (event) => {
        if (!dragging) return;
        event.preventDefault();
        event.stopPropagation();
        const now = performance.now();
        const dx = event.clientX - lastPointerX;
        const dy = event.clientY - lastPointerY;
        if (Math.abs(dx) + Math.abs(dy) > 1) didDrag = true;
        const dt = Math.max(16, now - lastPointerTime);
        rotationY += dx * 0.012;
        rotationX += dy * 0.012;
        velocityY = (dx / dt) * 0.34;
        velocityX = (dy / dt) * 0.34;
        velocityZ = ((dx - dy) / dt) * 0.07;
        lastPointerX = event.clientX;
        lastPointerY = event.clientY;
        lastPointerTime = now;
    };

    const stopDrag = (event) => {
        if (!dragging) return;
        dragging = false;
        canvas.classList.remove("dragging");
        if (event && canvas.hasPointerCapture(event.pointerId)) {
            canvas.releasePointerCapture(event.pointerId);
        }
    };

    resize();
    draw();
    animate();

    canvas.addEventListener("pointerdown", onPointerDown);
    canvas.addEventListener("pointermove", onPointerMove);
    canvas.addEventListener("pointerup", stopDrag);
    canvas.addEventListener("pointerleave", stopDrag);
    canvas.addEventListener("pointercancel", stopDrag);
    canvas.addEventListener("click", (event) => {
        if (didDrag) {
            event.preventDefault();
            event.stopPropagation();
            didDrag = false;
        }
    });
    window.addEventListener("resize", resize);
};

document.querySelectorAll(".icosa-shell").forEach(setupIcosaShell);
