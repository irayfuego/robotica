#!/usr/bin/env python3
"""
Headless simulation / preview tool for robot_eyes.
Renders all behaviors to PNG files and optionally shows a live window.

Usage:
  python simulate_preview.py              # render all behaviors to /tmp/robot_eyes_preview/
  python simulate_preview.py --live       # show live Tkinter window
  python simulate_preview.py --gif happy  # export behavior as animated GIF
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_eyes.eye_renderer import EyeRenderer, EyeState
from robot_eyes.animation_engine import AnimationEngine, lerp_state
from robot_eyes.behaviors import BehaviorLibrary, BEHAVIOR_MAP
from PIL import Image

OUT_DIR = "/tmp/robot_eyes_preview"


def render_all_behaviors():
    os.makedirs(OUT_DIR, exist_ok=True)
    renderer = EyeRenderer()
    print(f"Rendering {len(BEHAVIOR_MAP)} behaviors -> {OUT_DIR}")
    for name, factory in BEHAVIOR_MAP.items():
        anim = factory()
        lkfs = anim.left_keyframes
        rkfs = anim.right_keyframes or lkfs
        ls = lkfs[0].state if lkfs else EyeState()
        rs = rkfs[0].state if rkfs else EyeState()
        w = renderer.size
        combined = Image.new("RGB", (w * 2 + 10, w), (5, 5, 10))
        combined.paste(renderer.render(ls), (0, 0))
        combined.paste(renderer.render(rs), (w + 10, 0))
        out = os.path.join(OUT_DIR, f"{name}.png")
        combined.save(out)
        print(f"  OK {name:20s} -> {out}")
    print("Done.")


def export_gif(behavior_name, fps=20):
    factory = BEHAVIOR_MAP.get(behavior_name)
    if not factory:
        print(f"Unknown behavior '{behavior_name}'. Available: {list(BEHAVIOR_MAP.keys())}")
        sys.exit(1)
    os.makedirs(OUT_DIR, exist_ok=True)
    renderer = EyeRenderer()
    anim = factory()
    kfs  = anim.left_keyframes
    rkfs = anim.right_keyframes or kfs
    frames = []
    dt = 1.0 / fps
    for i, kf in enumerate(kfs):
        prev_l = kfs[i-1].state  if i > 0 else EyeState()
        ri     = min(i, len(rkfs)-1)
        rkf    = rkfs[ri]
        prev_r = rkfs[min(i-1, len(rkfs)-1)].state if i > 0 else EyeState()
        n = max(1, int(kf.duration / dt))
        for f in range(n):
            t  = f / max(n-1, 1)
            ls = lerp_state(prev_l, kf.state,  t, kf.easing)
            rs = lerp_state(prev_r, rkf.state, t, kf.easing)
            w  = renderer.size
            combined = Image.new("RGB", (w*2+10, w), (5, 5, 10))
            combined.paste(renderer.render(ls), (0, 0))
            combined.paste(renderer.render(rs), (w+10, 0))
            frames.append(combined)
        if kf.hold > 0:
            last = frames[-1]
            frames.extend([last.copy()] * max(1, int(kf.hold / dt)))
    out = os.path.join(OUT_DIR, f"{behavior_name}.gif")
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=int(dt*1000), loop=0)
    print(f"Saved: {out} ({len(frames)} frames @ {fps}fps)")


def live_preview(fps=30):
    try:
        import tkinter as tk
        from PIL import ImageTk
    except ImportError:
        print("Need tkinter: sudo apt install python3-tk")
        sys.exit(1)
    renderer = EyeRenderer()
    engine   = AnimationEngine(fps=fps)
    engine.start()
    names = list(BEHAVIOR_MAP.keys())
    idx   = [0]
    root  = tk.Tk()
    root.title("Robot Eyes Preview")
    root.configure(bg="#050510")
    w = renderer.size
    W, H = w*2+10, w
    canvas = tk.Canvas(root, width=W, height=H, bg="#050510", highlightthickness=0)
    canvas.pack(pady=5)
    lv = tk.StringVar(value="idle - click to cycle behaviors")
    tk.Label(root, textvariable=lv, fg="#88aaff", bg="#050510", font=("monospace", 11)).pack()
    tk.Label(root, text="Move mouse over canvas to control gaze",
             fg="#445566", bg="#050510", font=("monospace", 9)).pack(pady=2)
    photo = [None]; img_id = [None]
    def tick():
        ls, rs = engine.get_states()
        combined = Image.new("RGB", (W, H), (5, 5, 10))
        combined.paste(renderer.render(ls), (0, 0))
        combined.paste(renderer.render(rs), (w+10, 0))
        photo[0] = ImageTk.PhotoImage(combined)
        if img_id[0] is None:
            img_id[0] = canvas.create_image(0, 0, anchor=tk.NW, image=photo[0])
        else:
            canvas.itemconfig(img_id[0], image=photo[0])
        root.after(int(1000/fps), tick)
    def click(e):
        name = names[idx[0] % len(names)]; idx[0] += 1
        lv.set(f">> {name}")
        engine.play(BEHAVIOR_MAP[name]())
    def motion(e):
        gx = ((e.x - W/2) / (W/2)) * 0.8
        gy = ((e.y - H/2) / (H/2)) * 0.8
        engine.set_base_gaze(gx, gy)
    canvas.bind("<Button-1>", click)
    canvas.bind("<Motion>",   motion)
    tick()
    root.mainloop()
    engine.stop()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--live", action="store_true")
    p.add_argument("--gif",  type=str, metavar="BEHAVIOR")
    p.add_argument("--fps",  type=int, default=20)
    a = p.parse_args()
    if a.live:
        live_preview(fps=a.fps)
    elif a.gif:
        export_gif(a.gif, fps=a.fps)
    else:
        render_all_behaviors()

if __name__ == "__main__":
    main()
